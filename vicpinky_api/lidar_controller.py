import asyncio
import time
import threading
import logging

logger = logging.getLogger("VicPinkyLidar")


class LidarController:
    """
    A high-level controller for the Slamtec RPLIDAR C1, styled to match
    :class:`WheelController`.

    Wraps the **rplidarc1** library (``pip install rplidarc1``), which is an
    asyncio, C1-specific driver. The async scan loop is driven inside a private
    background thread that owns its own event loop, so callers use a simple
    synchronous, thread-safe API and never touch asyncio:

        lidar = LidarController(port="/dev/rplidar")
        lidar.connect()
        if lidar.is_obstacle_ahead(0.4):
            ...
        lidar.disconnect()

    Distances are exposed in **meters**; angles in **degrees [0, 360)** as
    reported by the sensor (0 deg is the sensor zero mark, adjust
    ``front_offset`` for how you mounted it). The cached scan is rebuilt once per
    full rotation so it never serves stale points.

    .. note::
        All rplidarc1-specific calls live in :meth:`_async_main` /
        :meth:`_consume_queue`. To swap drivers, reimplement those.
    """

    def __init__(self, port="/dev/rplidar", baudrate=460800,
                 front_offset=0.0, min_quality=0):
        self.port = port
        self.baudrate = baudrate  # RPLIDAR C1 = 460800 (NOT 115200!)
        self.front_offset = front_offset  # deg added to the "ahead" direction
        self.min_quality = min_quality

        self._lidar = None
        self._loop = None  # event loop owned by the background thread

        # Latest scan: {angle_int_deg: distance_m}, lowest distance kept per angle
        self._scan = {}

        # Threading & Control
        self._lock = threading.Lock()
        self._running = False
        self._thread = None

        # Safety / health
        self.is_connected = False
        self._error_count = 0
        self.MAX_ERRORS = 5
        self.last_scan_time = 0.0
        self.scan_count = 0

    # ------------------------------------------------------------------
    # Lifecycle
    # ------------------------------------------------------------------
    def connect(self):
        """Open the lidar and start the background scan thread."""
        self._running = True
        self.scan_count = 0
        self._error_count = 0
        self._thread = threading.Thread(target=self._run_loop, daemon=True)
        self._thread.start()

        # Wait for the device to init (healthcheck) and the first full rotation.
        start = time.time()
        while (time.time() - start) < 8.0:
            if not self._thread.is_alive():  # init failed and thread exited
                break
            if self.scan_count > 0:
                logger.info(f"Connected to RPLIDAR on {self.port}")
                return True
            time.sleep(0.05)

        logger.error("No scan received; check power / /dev/rplidar / baudrate.")
        self.disconnect()
        return False

    def _run_loop(self):
        """Background thread: owns a private asyncio event loop."""
        loop = asyncio.new_event_loop()
        self._loop = loop
        asyncio.set_event_loop(loop)
        try:
            loop.run_until_complete(self._async_main())
        except Exception as e:
            logger.error(f"Lidar loop crashed: {e}")
        finally:
            try:
                loop.close()
            except Exception:
                pass
            self._loop = None
            self.is_connected = False
            self._running = False

    async def _async_main(self):
        """Construct the driver (in this loop) and run scan + consumer tasks."""
        from rplidarc1 import RPLidar

        # Build the driver INSIDE the bg loop so its asyncio.Queue / Event bind
        # to this loop. __init__ runs the synchronous healthcheck and raises if
        # the device does not respond.
        try:
            self._lidar = RPLidar(self.port, self.baudrate)
        except Exception as e:
            logger.error(f"Lidar init failed on {self.port}: {e}")
            self._lidar = None
            return

        self.is_connected = True
        try:
            # simple_scan() sends the SCAN command and returns the reader
            # coroutine; we run it alongside our queue consumer.
            scan_coro = self._lidar.simple_scan(make_return_dict=False)
            await asyncio.gather(scan_coro, self._consume_queue())
        except Exception as e:
            logger.error(f"Lidar scan stopped: {e}")
        finally:
            try:
                self._lidar.stop_event.set()
            except Exception:
                pass

    async def _consume_queue(self):
        """Drain per-point results, rebuild the scan cache once per rotation."""
        q = self._lidar.output_queue
        ev = self._lidar.stop_event
        building = {}
        last_a = None

        while not ev.is_set() and self._running:
            try:
                item = await asyncio.wait_for(q.get(), timeout=1.0)
            except asyncio.TimeoutError:
                self._error_count += 1
                if self._error_count >= self.MAX_ERRORS:
                    logger.error("No lidar data; stopping read loop.")
                    ev.set()
                    break
                continue
            self._error_count = 0

            a_deg = item.get("a_deg")
            d_mm = item.get("d_mm")
            if a_deg is None:
                continue

            # A new rotation begins when the angle wraps back past 0 deg.
            if last_a is not None and a_deg < last_a - 180:
                if building:
                    with self._lock:
                        self._scan = building
                        self.last_scan_time = time.time()
                        self.scan_count += 1
                    building = {}
            last_a = a_deg

            if d_mm and d_mm > 0 and item.get("q", 0) >= self.min_quality:
                a = int(round(a_deg)) % 360
                d = d_mm / 1000.0  # mm -> m
                if a not in building or d < building[a]:
                    building[a] = d

    def disconnect(self):
        """Stop the background scan and release the lidar safely."""
        self._running = False

        # Ask the scan loop (running in the bg thread) to stop.
        loop, lidar = self._loop, self._lidar
        if loop is not None and lidar is not None:
            try:
                loop.call_soon_threadsafe(lidar.stop_event.set)
            except Exception:
                pass

        if self._thread:
            self._thread.join(timeout=3.0)

        # Send STOP (stops scanning + motor) and close the serial port.
        if self._lidar is not None:
            try:
                self._lidar.shutdown()
            except Exception:
                try:
                    self._lidar.reset()
                except Exception:
                    pass
            self._lidar = None

        self.is_connected = False
        logger.info("Lidar disconnected.")

    # ------------------------------------------------------------------
    # Data access (cheap, processed queries)
    # ------------------------------------------------------------------
    def get_scan_dict(self):
        """Returns a copy of the latest scan as {angle_deg_int: distance_m}."""
        with self._lock:
            return dict(self._scan)

    def get_scan(self):
        """Returns the latest scan as a sorted list of (angle_deg, distance_m)."""
        with self._lock:
            return sorted(self._scan.items())

    def get_distance_at(self, angle_deg, window=5):
        """
        Nearest distance (m) around a given direction, within +/- ``window`` deg.
        Returns ``float('inf')`` if nothing is detected there.
        """
        a = (angle_deg + self.front_offset) % 360
        return self.get_min_distance((a - window) % 360, (a + window) % 360)

    def get_min_distance(self, start_deg, end_deg):
        """
        Minimum distance (m) inside the angular sector [start_deg, end_deg].
        Handles wrap-around (e.g. 350 deg -> 10 deg across the 0 mark).
        Returns ``float('inf')`` if the sector is empty.
        """
        scan = self.get_scan_dict()
        wrap = start_deg > end_deg
        best = float('inf')
        for ang, dist in scan.items():
            in_sector = (ang >= start_deg or ang <= end_deg) if wrap \
                else (start_deg <= ang <= end_deg)
            if in_sector and dist < best:
                best = dist
        return best

    def get_closest(self):
        """Returns (angle_deg, distance_m) of the nearest point, or (None, inf)."""
        scan = self.get_scan_dict()
        if not scan:
            return None, float('inf')
        ang = min(scan, key=scan.get)
        return ang, scan[ang]

    def is_obstacle_ahead(self, distance=0.3, fov=30.0):
        """
        True if anything is closer than ``distance`` m within a ``fov``-deg cone
        centered on the front direction (front_offset applied).
        """
        half = fov / 2.0
        center = self.front_offset % 360
        return self.get_min_distance((center - half) % 360,
                                     (center + half) % 360) < distance

    def is_data_fresh(self, max_age=0.5):
        """True if a scan arrived within the last ``max_age`` seconds."""
        with self._lock:
            return self.scan_count > 0 and (time.time() - self.last_scan_time) < max_age

    def get_status(self):
        """Returns a status dictionary mirroring WheelController.get_status()."""
        ang, dist = self.get_closest()
        with self._lock:
            point_count = len(self._scan)
            age = time.time() - self.last_scan_time if self.scan_count else None
        return {
            'is_connected': self.is_connected,
            'scan_count': self.scan_count,
            'points_in_scan': point_count,
            'data_age_s': age,
            'closest': {'angle': ang, 'distance': dist},
            'obstacle_ahead': self.is_obstacle_ahead(),
            'error_count': self._error_count,
        }

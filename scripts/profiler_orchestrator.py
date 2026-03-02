#!/usr/bin/env python3
"""
Profiler Orchestrator Node
--------------------------
Exposes three aggregate services that fan out to all profiler nodes that are
currently live (i.e., their individual services are reachable):

  /start_acquisition_all   → calls /start_acquisition, /start_acquisition_2 …4
  /stop_acquisition_all    → calls /stop_acquisition,  /stop_acquisition_2  …4
  /trigger_software_all    → calls /trigger_software,  /trigger_software_2  …4

Only profilers whose service responds within the probe timeout are called.
"""

import rclpy
from rclpy.node import Node

from mecheye_profiler_ros_interface.srv import (
    StartAcquisition,
    StopAcquisition,
    TriggerSoftware,
)

# How long (seconds) to wait when checking whether a service is alive
PROBE_TIMEOUT = 0.5
CALL_TIMEOUT  = 5.0

PROFILER_SUFFIXES = ['', '_2', '_3', '_4']


class ProfilerOrchestrator(Node):
    def __init__(self):
        super().__init__('profiler_orchestrator')

        # Build service clients for every profiler slot
        self._start_clients   = {}
        self._stop_clients    = {}
        self._trigger_clients = {}

        for suffix in PROFILER_SUFFIXES:
            self._start_clients[suffix]   = self.create_client(
                StartAcquisition, f'start_acquisition{suffix}')
            self._stop_clients[suffix]    = self.create_client(
                StopAcquisition,  f'stop_acquisition{suffix}')
            self._trigger_clients[suffix] = self.create_client(
                TriggerSoftware,  f'trigger_software{suffix}')

        # Aggregate services
        self.create_service(StartAcquisition, 'start_acquisition_all',
                            self._start_all_callback)
        self.create_service(StopAcquisition,  'stop_acquisition_all',
                            self._stop_all_callback)
        self.create_service(TriggerSoftware,  'trigger_software_all',
                            self._trigger_all_callback)

        self.get_logger().info('Profiler Orchestrator ready.')

    # ── helpers ────────────────────────────────────────────────────────────

    def _live_clients(self, client_dict):
        """Return only those clients whose service is currently reachable."""
        live = []
        for suffix, client in client_dict.items():
            if client.wait_for_service(timeout_sec=PROBE_TIMEOUT):
                live.append((suffix, client))
            else:
                self.get_logger().debug(
                    f'Service not reachable (suffix="{suffix}") — skipping.')
        return live

    def _call_all(self, client_dict, request):
        """Fan out request to all live clients; return aggregated result."""
        live = self._live_clients(client_dict)
        if not live:
            self.get_logger().warn('No profiler services are reachable!')

        results = []
        futures = [(suffix, client.call_async(type(request)()))
                   for suffix, client in live]

        # Spin until all futures complete
        for suffix, future in futures:
            rclpy.spin_until_future_complete(self, future,
                                             timeout_sec=CALL_TIMEOUT)
            if future.result() is not None:
                r = future.result()
                self.get_logger().info(
                    f'Profiler{suffix}: error_code={r.error_code} '
                    f'"{r.error_description}"')
                results.append(r)
            else:
                self.get_logger().error(
                    f'Profiler{suffix}: call timed out or failed.')

        # Return the last result (or a blank one if nothing responded)
        if results:
            return results[-1]
        blank = type(request).Response()
        blank.error_code = -1
        blank.error_description = 'No profilers responded.'
        return blank

    # ── service callbacks ──────────────────────────────────────────────────

    def _start_all_callback(self, request, response):
        self.get_logger().info('start_acquisition_all called')
        result = self._call_all(self._start_clients, request)
        response.error_code        = result.error_code
        response.error_description = result.error_description
        return response

    def _stop_all_callback(self, request, response):
        self.get_logger().info('stop_acquisition_all called')
        result = self._call_all(self._stop_clients, request)
        response.error_code        = result.error_code
        response.error_description = result.error_description
        return response

    def _trigger_all_callback(self, request, response):
        self.get_logger().info('trigger_software_all called')
        result = self._call_all(self._trigger_clients, request)
        response.error_code        = result.error_code
        response.error_description = result.error_description
        return response


def main():
    rclpy.init()
    node = ProfilerOrchestrator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

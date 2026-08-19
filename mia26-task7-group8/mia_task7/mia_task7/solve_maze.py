#!/usr/bin/env python3
"""High-level controller for the maze run.

This node owns the entire sequence. Each stage is a plain class that takes
this node plus the shared gate-service client, exposes ``async run() -> bool``,
and drives the robot only through the move_x / move_yaw action clients.
Stages never talk to each other and never publish to /cmd_vel themselves --
ordering and failure handling live here, in :meth:`SolveMaze.solve_maze`.

Threading model
---------------
The stages are coroutines that await rclpy futures, but rclpy has no asyncio
event loop of its own, so:

* the node is spun by a ``MultiThreadedExecutor`` on a background thread --
  that is what actually completes those futures;
* ``solve_maze()`` runs on an asyncio loop on the main thread.

A ``ReentrantCallbackGroup`` lets service and action callbacks fire while a
stage is still awaiting an earlier one; with the default group they would
serialise and deadlock.
"""

import asyncio
import importlib
import threading
import traceback

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.task import Future
from std_srvs.srv import SetBool


def _make_rclpy_futures_awaitable():
    """Let asyncio await rclpy Futures.

    rclpy's Future.__await__ does `yield self`, but an asyncio Task only
    accepts a bare `yield` or an asyncio Future and raises "Task got bad
    yield" on anything else. The stages await rclpy futures directly, so
    without this every stage dies on its first service or action call.

    Polling on asyncio.sleep rather than yielding bare keeps this from
    becoming a busy loop -- the executor thread is what actually completes
    the future, and 10 ms is well inside the 20 Hz control loop.
    """
    def __await__(self):
        while not self.done():
            yield from asyncio.sleep(0.01).__await__()
        return self.result()

    Future.__await__ = __await__


_make_rclpy_futures_awaitable()

# One service moves both gates, in opposite directions. See the repo README.
DEFAULT_GATE_SERVICE = 'toggle_walls_1_2'

# Gazebo needs a moment after launch before the service is advertised.
DEFAULT_STARTUP_TIMEOUT_S = 30.0

# (label, module, class) in the order they must run.
STAGE_SPECS = (
    ('Stage 1', 'mia_task7.stage1', 'Stage1'),
    ('Stage 2', 'mia_task7.stage2', 'Stage2'),
    ('Stage 3', 'mia_task7.stage3', 'Stage3'),
)


class SolveMaze(Node):
    """Orchestrates the stages that carry the robot from start to finish."""

    def __init__(self):
        super().__init__('solve_maze')

        self.declare_parameter('gate_service', DEFAULT_GATE_SERVICE)
        self.declare_parameter('startup_timeout', DEFAULT_STARTUP_TIMEOUT_S)

        gate_service = self.get_parameter(
            'gate_service').get_parameter_value().string_value
        self.startup_timeout = self.get_parameter(
            'startup_timeout').get_parameter_value().double_value

        # Every stage shares this one client and callback group, so a stage
        # waiting on the gate cannot block a sibling action callback.
        self.callback_group = ReentrantCallbackGroup()
        self.gate_client = self.create_client(
            SetBool, gate_service, callback_group=self.callback_group)

        self.stages = self._load_stages()

    def _load_stages(self):
        """Return the stages that are actually implemented, in run order.

        Stages are still being written by different people, so an unfinished
        one is reported and skipped rather than taking down the whole run --
        that way stage 1 stays demonstrable while stages 2 and 3 land.
        """
        stages = []

        for label, module_name, class_name in STAGE_SPECS:
            try:
                module = importlib.import_module(module_name)
            except ImportError as exc:
                self.get_logger().warn(
                    f'{label}: no module {module_name} yet, skipping ({exc})')
                continue

            stage_cls = getattr(module, class_name, None)
            if stage_cls is None:
                self.get_logger().warn(
                    f'{label}: {module_name} defines no class {class_name}, '
                    f'skipping')
                continue

            try:
                stage = stage_cls(self, self.gate_client)
            except TypeError as exc:
                self.get_logger().warn(
                    f'{label}: {class_name} does not accept '
                    f'(node, gate_client), skipping ({exc})')
                continue

            if not callable(getattr(stage, 'run', None)):
                self.get_logger().warn(
                    f'{label}: {class_name} has no run() method, skipping. '
                    f'Check that run() is indented inside the class.')
                continue

            stages.append((label, stage))

        return stages

    async def solve_maze(self) -> bool:
        """Drive the robot from the start pad to the finish pad.

        Calls each stage in order and stops at the first failure. Returns
        True only if every loaded stage succeeded.
        """
        if not self.stages:
            self.get_logger().error('No runnable stages found, nothing to do.')
            return False

        loaded = ', '.join(label for label, _ in self.stages)
        self.get_logger().info(f'Maze run starting. Stages loaded: {loaded}.')

        if len(self.stages) < len(STAGE_SPECS):
            self.get_logger().warn(
                f'Only {len(self.stages)} of {len(STAGE_SPECS)} stages are '
                f'available -- the robot will stop short of the finish pad.')

        # The stages each use a short per-call timeout, so wait for the
        # simulation to come up here rather than failing stage 1 on startup.
        self.get_logger().info('Waiting for the gate service...')
        if not self.gate_client.wait_for_service(
                timeout_sec=self.startup_timeout):
            self.get_logger().error(
                f'Gate service {self.gate_client.srv_name} never appeared '
                f'after {self.startup_timeout:.0f}s. Is the maze running?')
            return False

        for label, stage in self.stages:
            self.get_logger().info(f'--- {label} starting ---')
            try:
                succeeded = await stage.run()
            except Exception:
                self.get_logger().error(
                    f'{label} raised:\n{traceback.format_exc()}')
                return False

            if not succeeded:
                self.get_logger().error(f'{label} failed. Aborting the run.')
                return False

            self.get_logger().info(f'--- {label} complete ---')

        self.get_logger().info('Maze run complete.')
        return True


def main(args=None):
    rclpy.init(args=args)
    node = SolveMaze()

    # The executor completes the futures the stages await, so it has to run
    # somewhere other than the thread hosting the asyncio loop.
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    succeeded = False
    try:
        succeeded = asyncio.run(node.solve_maze())
    except KeyboardInterrupt:
        node.get_logger().info('Interrupted.')
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    return 0 if succeeded else 1


if __name__ == '__main__':
    main()

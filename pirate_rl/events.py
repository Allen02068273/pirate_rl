import heapq
from collections.abc import Callable
from dataclasses import dataclass, field


class EventQueue:
    def __init__(self):
        self.events: list[ScheduledEvent] = []
        self.sequence = 0  # maintains FIFO order for simultaneous events

    def schedule(self, time: float, callback: Callable) -> None:
        event = ScheduledEvent(time=time, sequence=self.sequence, callback=callback)
        heapq.heappush(self.events, event)
        self.sequence += 1

    def pop_ready(self, current_time: float):
        while self.events and current_time >= self.events[0].time:
            event = heapq.heappop(self.events)
            event.callback()


@dataclass(order=True)
class ScheduledEvent:
    time: float
    sequence: int  # maintains FIFO order for simultaneous events
    callback: Callable = field(compare=False)

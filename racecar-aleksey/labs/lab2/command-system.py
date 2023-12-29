import abc

class Command(abc.ABC):
    @abc.abstractmethod
    def run_command(self):
        pass

    @abc.abstractmethod
    def is_finished(self) -> bool:
        pass

    @abc.abstractmethod
    def end(self):
        pass


class Scheduler:
    def __init__(self):
        self.queue = []

    def add_command(self, command: Command):
        self.queue.append(command)

    def run_scheduler(self):
        if len(self.queue):
            command = self.queue[0]
            command.run_command()
            if command.is_finished():
                command.end()
                self.queue.pop(0)

    def cancel_command_idx(self, idx):
        self.queue[idx].end()
        self.queue.pop(idx)

    def cancel_all(self):
        self.queue = []

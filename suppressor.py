import os
import sys

from contextlib import contextmanager


# Suppress output on 'stdout'
@contextmanager
def suppress_stdout():
    with open(os.devnull, "w") as devnull:
        old_stdout = sys.stdout
        sys.stdout = devnull
        try:
            yield
        finally:
            sys.stdout = old_stdout


# Suppress output on 'stderr'
@contextmanager
def suppress_stderr():
    with open(os.devnull, "w") as devnull:
        old_stderr = sys.stderr
        sys.stderr = devnull
        try:
            yield
        finally:
            sys.stderr = old_stderr

class SuppressNativePrints:
    def __enter__(self):
        self._stdout_fd = sys.stdout.fileno()
        self._stderr_fd = sys.stderr.fileno()

        self._saved_stdout = os.dup(self._stdout_fd)
        self._saved_stderr = os.dup(self._stderr_fd)

        self._devnull = os.open(os.devnull, os.O_WRONLY)
        os.dup2(self._devnull, self._stdout_fd)
        os.dup2(self._devnull, self._stderr_fd)

    def __exit__(self, exc_type, exc_val, exc_tb):
        os.dup2(self._saved_stdout, self._stdout_fd)
        os.dup2(self._saved_stderr, self._stderr_fd)
        os.close(self._devnull)
        os.close(self._saved_stdout)
        os.close(self._saved_stderr)

import threading
import time
from types import SimpleNamespace

from sphero_rvr_driver.collision_stop_node import DriverServiceForwarder


class FakeFuture:
    def __init__(self):
        self._callbacks = []
        self._done = False
        self._result = None
        self._exception = None
        self.cancelled = False
        self._lock = threading.Lock()

    def add_done_callback(self, callback):
        with self._lock:
            if self._done:
                call_now = True
            else:
                self._callbacks.append(callback)
                call_now = False
        if call_now:
            callback(self)

    def set_result(self, result):
        self._finish(result=result)

    def set_exception(self, exc):
        self._finish(exception=exc)

    def cancel(self):
        self.cancelled = True
        self._finish(exception=RuntimeError("cancelled"))

    def result(self):
        if self._exception is not None:
            raise self._exception
        return self._result

    def _finish(self, *, result=None, exception=None):
        with self._lock:
            if self._done:
                return
            self._done = True
            self._result = result
            self._exception = exception
            callbacks = list(self._callbacks)
            self._callbacks.clear()
        for callback in callbacks:
            callback(self)


class FakeClient:
    def __init__(self, future=None, *, ready=True, call_exc=None):
        self.future = future or FakeFuture()
        self.ready = ready
        self.call_exc = call_exc
        self.requests = []

    def service_is_ready(self):
        return self.ready

    def call_async(self, request):
        if self.call_exc is not None:
            raise self.call_exc
        self.requests.append(request)
        return self.future


def _request():
    return object()


def test_driver_success_is_propagated_after_downstream_confirmation():
    future = FakeFuture()
    client = FakeClient(future)
    forwarder = DriverServiceForwarder(timeout_s=0.5)

    thread = threading.Thread(target=lambda: time.sleep(0.01) or future.set_result(SimpleNamespace(success=True, message="driver stopped")))
    thread.start()
    result = forwarder.call(client, "driver stop", _request)
    thread.join(timeout=1)

    assert result.success is True
    assert result.message == "driver stopped"
    assert len(client.requests) == 1


def test_driver_rejection_is_propagated_as_failure():
    future = FakeFuture()
    client = FakeClient(future)
    forwarder = DriverServiceForwarder(timeout_s=0.5)

    thread = threading.Thread(target=lambda: time.sleep(0.01) or future.set_result(SimpleNamespace(success=False, message="estop rejected")))
    thread.start()
    result = forwarder.call(client, "driver estop", _request)
    thread.join(timeout=1)

    assert result.success is False
    assert result.message == "estop rejected"


def test_unavailable_driver_client_fails_without_queuing_request():
    client = FakeClient(ready=False)
    forwarder = DriverServiceForwarder(timeout_s=0.5)

    result = forwarder.call(client, "driver stop", _request)

    assert result.success is False
    assert "unavailable" in result.message
    assert client.requests == []


def test_async_driver_exception_is_reported_as_failure():
    future = FakeFuture()
    client = FakeClient(future)
    forwarder = DriverServiceForwarder(timeout_s=0.5)

    thread = threading.Thread(target=lambda: time.sleep(0.01) or future.set_exception(RuntimeError("wire broke")))
    thread.start()
    result = forwarder.call(client, "driver clear_estop", _request)
    thread.join(timeout=1)

    assert result.success is False
    assert "response failed" in result.message
    assert "wire broke" in result.message


def test_downstream_timeout_is_bounded_and_cancels_pending_future():
    future = FakeFuture()
    client = FakeClient(future)
    forwarder = DriverServiceForwarder(timeout_s=0.01)

    start = time.monotonic()
    result = forwarder.call(client, "driver stop", _request)
    elapsed = time.monotonic() - start

    assert result.success is False
    assert "timed out" in result.message
    assert elapsed < 0.25
    assert future.cancelled is True


def test_pending_future_during_shutdown_returns_failure_and_is_cancelled():
    future = FakeFuture()
    client = FakeClient(future)
    forwarder = DriverServiceForwarder(timeout_s=5.0)
    holder = {}

    thread = threading.Thread(target=lambda: holder.setdefault("result", forwarder.call(client, "driver estop", _request)))
    thread.start()
    while not client.requests:
        time.sleep(0.001)

    forwarder.shutdown()
    thread.join(timeout=1)

    assert thread.is_alive() is False
    assert holder["result"].success is False
    assert "shutdown" in holder["result"].message
    assert future.cancelled is True

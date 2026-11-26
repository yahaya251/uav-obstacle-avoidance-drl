"""Clock and read-only HTTP contracts without needing ROS or network access."""
from collections import deque
import importlib.util
import io
import json
from pathlib import Path
import time
import unittest
from unittest.mock import patch

project=Path(__file__).resolve().parents[1]
source = project / 'simulation/monitor.py'
spec = importlib.util.spec_from_file_location('uav_nav_passive_monitor', source)
monitor=importlib.util.module_from_spec(spec)
spec.loader.exec_module(monitor)


class FakeSocket:
    def __init__(self,request):
        self.request=request
        self.response=bytearray()
    def makefile(self,*args,**kwargs):
        return io.BytesIO(self.request)
    def sendall(self,data):
        self.response.extend(data)


class MonitorContractTests(unittest.TestCase):
    def test_progress_clock_repeated_timestamp_does_not_refresh_source(self):
        progress=monitor.SourceProgress()
        progress.received(12345.,10.)
        progress.received(12345.,10.8)
        self.assertAlmostEqual(progress.age(10.8),.8)
        progress.received(12345.1,10.9)
        self.assertAlmostEqual(progress.age(10.9),0.)

    def test_simulation_clock_reset_is_recorded_without_mixed_clock_subtraction(self):
        progress=monitor.SourceProgress()
        progress.received(400.,10.)
        progress.received(0.,10.1)
        self.assertEqual(progress.resets,1)
        self.assertAlmostEqual(progress.age(10.4),.3)

    def test_status_snapshot_isolation_and_expiring_server_age(self):
        store = monitor.StatusStore()
        store.update({'observation': {'ready': True}, 'flight_command_sent': False}, 10.0)
        with patch.object(monitor.time, 'monotonic', return_value=10.7) as clock:
            first = store.snapshot()
            first['observation']['ready'] = False
            clock.return_value = 11.0
            second = store.snapshot()

        self.assertTrue(second['observation']['ready'])
        self.assertAlmostEqual(first['monitor_sample_age_s'], 0.7)
        self.assertAlmostEqual(second['monitor_sample_age_s'], 1.0)

    def request(self,path,method='GET'):
        store=monitor.StatusStore()
        store.update({'flight_command_sent':False,'observation':{'ready':False}},time.monotonic())
        handler=monitor.make_handler(store,'<h1>Real monitor page</h1>')
        socket=FakeSocket(f'{method} {path} HTTP/1.0\r\nHost: localhost\r\n\r\n'.encode())
        handler(socket,('127.0.0.1',12345),object())
        return bytes(socket.response).split(b'\r\n\r\n',1)

    def test_api_status_is_read_only_finite_and_not_cacheable(self):
        headers,body=self.request('/api/status')
        self.assertIn(b'200 OK',headers)
        self.assertIn(b'no-store',headers)
        payload=json.loads(body)
        self.assertFalse(payload['flight_command_sent'])
        self.assertGreaterEqual(payload['monitor_sample_age_s'],0.)

    def test_dashboard_route_and_unknown_file_path(self):
        headers,body=self.request('/')
        self.assertIn(b'200 OK',headers)
        self.assertIn(b'Real monitor page',body)
        headers,_=self.request('/../../etc/passwd')
        self.assertIn(b'404 Not Found',headers)

    def test_write_methods_are_rejected(self):
        for method in ('POST','PUT','PATCH','DELETE'):
            with self.subTest(method=method):
                headers,body=self.request('/api/status',method)
                self.assertIn(b'405 Method Not Allowed',headers)
                self.assertEqual(json.loads(body)['error'],'read_only_monitor')

    def test_frequency_window_retires_stopped_stream(self):
        history=deque([1.,1.1,1.2])
        self.assertAlmostEqual(monitor.frequency(history,1.3),2/.3)
        self.assertEqual(monitor.frequency(history,7.),0.)


if __name__=='__main__':
    unittest.main(verbosity=2)

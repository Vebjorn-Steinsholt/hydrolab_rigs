"""
Tests for the web server's HTTP side: keep-alive for the page's requests (TODO 49).

The server runs for real on a free local port; the ROS node behind it is real too, but nothing
else runs. Needs the workspace sourced (rig_interfaces, nav2_msgs).

    cd ~/ros_ws/src/hydrolab_rigs
    python3 -m pytest rig_web/test/test_web_http.py -v
"""

import http.client
import importlib.util
import json
import os
import pathlib
import threading

import pytest

os.environ.setdefault('ROS_AUTOMATIC_DISCOVERY_RANGE', 'LOCALHOST')
os.environ['ROS_DOMAIN_ID'] = str(100 + os.getpid() % 100)  # never the lab's domain

rclpy = pytest.importorskip('rclpy')

SCRIPT = pathlib.Path(__file__).resolve().parents[1] / 'scripts' / 'web_server.py'
_spec = importlib.util.spec_from_file_location('web_server', SCRIPT)
web_server = importlib.util.module_from_spec(_spec)
try:
    _spec.loader.exec_module(web_server)
except ModuleNotFoundError as error:  # rig_interfaces or nav2_msgs not sourced
    pytest.skip(f'workspace not sourced: {error}', allow_module_level=True)

HTML = pathlib.Path(__file__).resolve().parents[1] / 'html'


@pytest.fixture
def server():
    """Serve a WebNode on 127.0.0.1 and a free port; return (node, port)."""
    rclpy.init(args=['web', '--ros-args', '-p', 'idle_timeout_s:=0.0'])
    node = web_server.WebNode()
    httpd = web_server.ThreadingHTTPServer(
        ('127.0.0.1', 0), web_server.make_handler(node, HTML))
    httpd.daemon_threads = True
    threading.Thread(target=httpd.serve_forever, daemon=True).start()
    yield node, httpd.server_address[1]
    httpd.shutdown()
    httpd.server_close()
    node.destroy_node()
    rclpy.try_shutdown()


def post(conn, path, body=b'{}'):
    conn.request('POST', path, body=body, headers={'Content-Type': 'application/json'})
    response = conn.getresponse()
    return response.status, response.read()


def test_acknowledgments_reuse_one_connection(server):
    node, port = server
    conn = http.client.HTTPConnection('127.0.0.1', port, timeout=5)
    local_ports = set()
    for _ in range(20):
        conn.request('POST', '/api/ack', body=b'{}', headers={'Content-Type': 'application/json'})
        local_ports.add(conn.sock.getsockname()[1])  # a new connection gets a new local port
        response = conn.getresponse()
        assert (response.status, response.read()) == (204, b'')
        assert not response.will_close
    conn.close()
    assert len(local_ports) == 1  # never reconnected
    assert node._last_ack_from == '127.0.0.1'


def test_a_body_left_unread_would_break_the_next_request(server):
    # stop and keep_running ignore their '{}' body; it must still be read.
    _, port = server
    conn = http.client.HTTPConnection('127.0.0.1', port, timeout=5)
    for path in ('/api/stop', '/api/keep_running', '/api/ack', '/api/stop'):
        conn.request('POST', path, body=b'{}', headers={'Content-Type': 'application/json'})
        response = conn.getresponse()
        status, body = response.status, response.read()
        assert status in (200, 204), path
        assert not response.will_close, path
        if status == 200:
            assert 'ok' in json.loads(body)
    conn.close()


def test_start_still_reads_height_and_period(server):
    _, port = server
    conn = http.client.HTTPConnection('127.0.0.1', port, timeout=5)
    status, body = post(conn, '/api/start', b'{"height": "x", "period": 2}')
    assert status == 200
    assert json.loads(body) == {'ok': False, 'message': 'Send height and period as numbers.'}
    assert post(conn, '/api/ack') == (204, b'')  # same connection, still in step
    conn.close()


def test_get_files_on_a_kept_connection(server):
    _, port = server
    conn = http.client.HTTPConnection('127.0.0.1', port, timeout=5)
    for path in ('/', '/app.js', '/api/config'):
        conn.request('GET', path)
        response = conn.getresponse()
        assert response.status == 200, path
        assert int(response.getheader('Content-Length')) == len(response.read())
    conn.close()


def test_event_stream_closes_its_own_connection(server):
    _, port = server
    conn = http.client.HTTPConnection('127.0.0.1', port, timeout=5)
    conn.request('GET', '/api/events')
    response = conn.getresponse()
    assert response.status == 200
    assert response.getheader('Connection') == 'close'
    assert response.getheader('Content-Type') == 'text/event-stream'
    assert response.fp.readline().startswith(b'data: {')
    conn.close()

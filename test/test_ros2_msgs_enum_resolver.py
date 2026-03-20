# Copyright 2024 Your Name
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Python tests for ros2_msgs_enum_resolver.EnumResolver."""

import os
import threading

import pytest

from ros2_msgs_enum_resolver import EnumResolver


@pytest.fixture(autouse=True)
def fake_ament_index(tmp_path, monkeypatch):
    """Set up a fake ament_index in a temp directory."""
    pkg = tmp_path / 'share' / 'fake_pkg'
    for d in ('msg', 'srv', 'action'):
        (pkg / d).mkdir(parents=True)

    idx = tmp_path / 'share' / 'ament_index' / 'resource_index' / 'packages'
    idx.mkdir(parents=True)
    (idx / 'fake_pkg').touch()

    (pkg / 'msg' / 'State.msg').write_text(
        'uint8 NONE      = 0\n'
        'uint8 INIT      = 1\n'
        'uint8 STARTING  = 1\n'
        'uint8 CONNECTED = 2\n'
        'uint8 ERROR     = 255\n'
        'uint8 HEX_VAL   = 0xFF\n'
        'uint8 state\n'
    )
    (pkg / 'srv' / 'SetMode.srv').write_text(
        'uint8 MODE_A = 10\n'
        'uint8 MODE_B = 20\n'
        'uint8 mode\n'
        '---\n'
        'uint8 STATUS_OK  = 0\n'
        'uint8 STATUS_ERR = 1\n'
        'bool success\n'
    )
    (pkg / 'action' / 'Navigate.action').write_text(
        'uint8 PRIORITY_LOW  = 0\n'
        'uint8 PRIORITY_HIGH = 1\n'
        'float64 target_x\n'
        '---\n'
        'uint8 RESULT_OK   = 0\n'
        'uint8 RESULT_FAIL = 1\n'
        'bool reached\n'
        '---\n'
        'uint8 FB_MOVING = 0\n'
        'uint8 FB_STUCK  = 1\n'
        'float64 progress\n'
    )

    orig = os.environ.get('AMENT_PREFIX_PATH', '')
    monkeypatch.setenv(
        'AMENT_PREFIX_PATH',
        str(tmp_path) + (':' + orig if orig else '')
    )
    EnumResolver.clear_cache()
    yield
    EnumResolver.clear_cache()


def test_msg_single():
    """Test single-name forward resolution."""
    assert EnumResolver.resolve('fake_pkg/msg/State', 0) == ['NONE']


def test_msg_alias():
    """Test aliased value forward resolution."""
    assert EnumResolver.resolve('fake_pkg/msg/State', 1) == ['INIT', 'STARTING']


def test_msg_hex_alias():
    """Test hex constant alias."""
    assert EnumResolver.resolve('fake_pkg/msg/State', 255) == ['ERROR', 'HEX_VAL']


def test_msg_missing():
    """Test missing value returns None."""
    assert EnumResolver.resolve('fake_pkg/msg/State', 99) is None


def test_msg_reverse():
    """Test reverse resolution."""
    assert EnumResolver.reverse_resolve('fake_pkg/msg/State', 'NONE') == 0
    assert EnumResolver.reverse_resolve('fake_pkg/msg/State', 'INIT') == 1
    assert EnumResolver.reverse_resolve('fake_pkg/msg/State', 'STARTING') == 1


def test_msg_reverse_missing():
    """Test missing name returns None."""
    assert EnumResolver.reverse_resolve('fake_pkg/msg/State', 'UNKNOWN') is None


def test_get_constants_keys():
    """Test get_constants returns correct keys."""
    c = EnumResolver.get_constants('fake_pkg/msg/State')
    assert set(c.keys()) == {0, 1, 2, 255}


def test_get_constants_alias():
    """Test get_constants returns alias list."""
    c = EnumResolver.get_constants('fake_pkg/msg/State')
    assert c[1] == ['INIT', 'STARTING']


def test_get_constants_sorted():
    """Test get_constants returns sorted keys."""
    c = EnumResolver.get_constants('fake_pkg/msg/State')
    assert list(c.keys()) == sorted(c.keys())


def test_unambiguous_false():
    """Test is_unambiguous returns False for aliased type."""
    assert EnumResolver.is_unambiguous('fake_pkg/msg/State') is False


def test_unambiguous_true():
    """Test is_unambiguous returns True for unaliased type."""
    assert EnumResolver.is_unambiguous('fake_pkg/srv/SetMode/Request') is True


def test_srv_request():
    """Test srv Request section."""
    assert EnumResolver.resolve('fake_pkg/srv/SetMode/Request', 10) == ['MODE_A']
    assert EnumResolver.resolve('fake_pkg/srv/SetMode/Request', 0) is None


def test_srv_response():
    """Test srv Response section."""
    assert EnumResolver.resolve('fake_pkg/srv/SetMode/Response', 0) == ['STATUS_OK']
    assert EnumResolver.resolve('fake_pkg/srv/SetMode/Response', 1) == ['STATUS_ERR']


def test_srv_default_is_request():
    """Test srv default section is Request."""
    assert EnumResolver.resolve('fake_pkg/srv/SetMode', 10) == ['MODE_A']


def test_action_goal():
    """Test action Goal section."""
    assert EnumResolver.resolve('fake_pkg/action/Navigate/Goal', 0) == ['PRIORITY_LOW']
    assert EnumResolver.resolve('fake_pkg/action/Navigate/Goal', 1) == ['PRIORITY_HIGH']


def test_action_result():
    """Test action Result section."""
    assert EnumResolver.resolve('fake_pkg/action/Navigate/Result', 0) == ['RESULT_OK']
    assert EnumResolver.resolve('fake_pkg/action/Navigate/Result', 1) == ['RESULT_FAIL']


def test_action_feedback():
    """Test action Feedback section."""
    assert EnumResolver.resolve('fake_pkg/action/Navigate/Feedback', 0) == ['FB_MOVING']
    assert EnumResolver.resolve('fake_pkg/action/Navigate/Feedback', 1) == ['FB_STUCK']


def test_is_available():
    """Test is_available for existing and missing types."""
    assert EnumResolver.is_available('fake_pkg/msg/State') is True
    assert EnumResolver.is_available('ghost_pkg/msg/Whatever') is False


def test_bad_type_string():
    """Test bad type string returns None/False."""
    assert EnumResolver.is_available('no_slashes') is False
    assert EnumResolver.resolve('no_slashes', 0) is None


def test_clear_cache_and_reload():
    """Test cache clears and reloads correctly."""
    EnumResolver.preload('fake_pkg/msg/State')
    EnumResolver.clear_cache()
    assert EnumResolver.resolve('fake_pkg/msg/State', 0) == ['NONE']


def test_preload():
    """Test preload returns correct booleans."""
    assert EnumResolver.preload('fake_pkg/msg/State') is True
    assert EnumResolver.preload('ghost_pkg/msg/Ghost') is False


def test_thread_safety():
    """Test concurrent access is thread-safe."""
    results, errors = [], []

    def worker():
        try:
            results.append(EnumResolver.resolve('fake_pkg/msg/State', 1))
        except Exception as e:
            errors.append(e)

    threads = [threading.Thread(target=worker) for _ in range(20)]
    for t in threads:
        t.start()
    for t in threads:
        t.join()

    assert not errors
    assert all(r == ['INIT', 'STARTING'] for r in results)
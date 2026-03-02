import pytest
from lerobot.teleoperators.csvarm_leader import CsvArmLeader, CsvArmLeaderConfig

def test_csvarm_leader_instantiation():
    config = CsvArmLeaderConfig(port="non_existent_port.csv")
    leader = CsvArmLeader(config)
    assert leader.name == "csvarm_leader"
    assert not leader.is_connected

def test_csvarm_leader_connect():
    config = CsvArmLeaderConfig(port="non_existent_port.csv")
    leader = CsvArmLeader(config)
    leader.connect()
    assert leader.is_connected

def test_csvarm_leader_get_action_no_file():
    config = CsvArmLeaderConfig(port="non_existent_port.csv")
    leader = CsvArmLeader(config)
    leader.connect()
    # Should not crash and return default joint values (reset_deg)
    action = leader.get_action()
    assert "shoulder_pan.pos" in action
    assert action["shoulder_pan.pos"] == 0.0
    assert action["shoulder_lift.pos"] == -90.0

#!/bin/bash
set -e

echo "📦 Creating /etc/cyclonedds/ directory..."
sudo mkdir -p /etc/cyclonedds

echo "📝 Writing CycloneDDS config for robot..."
cat <<EOF | sudo tee /etc/cyclonedds/cyclonedds_robot.xml > /dev/null
<CycloneDDS>
  <Domain>
    <General>
      <AllowMulticast>false</AllowMulticast>
    </General>
    <Discovery>
      <Peers>
        <Peer address="192.168.1.6"/> <!-- PC (WSL2) -->
      </Peers>
      <ParticipantIndex>auto</ParticipantIndex>
    </Discovery>
  </Domain>
</CycloneDDS>
EOF

echo "🔧 Updating ~/.bashrc for ROS and CycloneDDS..."
ROS_LINE='source /opt/ros/humble/setup.bash'
DDS_LINE='export CYCLONEDDS_URI=file:///etc/cyclonedds/cyclonedds_robot.xml'

grep -qxF "$ROS_LINE" ~/.bashrc || echo "$ROS_LINE" >> ~/.bashrc
grep -qxF "$DDS_LINE" ~/.bashrc || echo "$DDS_LINE" >> ~/.bashrc

echo "✅ Robot setup done. You can now run: source ~/.bashrc"

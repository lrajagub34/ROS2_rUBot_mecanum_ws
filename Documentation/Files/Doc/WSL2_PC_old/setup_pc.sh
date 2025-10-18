#!/bin/bash
set -e

echo "📦 Creating /etc/cyclonedds/ directory..."
sudo mkdir -p /etc/cyclonedds

echo "📝 Writing CycloneDDS config for PC..."
cat <<EOF | sudo tee /etc/cyclonedds/cyclonedds_pc.xml > /dev/null
<CycloneDDS>
  <Domain>
    <General>
      <AllowMulticast>false</AllowMulticast>
    </General>
    <Discovery>
      <Peers>
        <Peer address="192.168.1.54"/> <!-- Robot -->
      </Peers>
      <ParticipantIndex>auto</ParticipantIndex>
    </Discovery>
  </Domain>
</CycloneDDS>
EOF

echo "🔧 Updating ~/.bashrc for ROS, CycloneDDS and X11..."
ROS_LINE='source /opt/ros/humble/setup.bash'
DDS_LINE='export CYCLONEDDS_URI=file:///etc/cyclonedds/cyclonedds_pc.xml'
DISP_LINE='export DISPLAY=:0'
GL_LINE='export LIBGL_ALWAYS_INDIRECT=1'

for LINE in "$ROS_LINE" "$DDS_LINE" "$DISP_LINE" "$GL_LINE"; do
  grep -qxF "$LINE" ~/.bashrc || echo "$LINE" >> ~/.bashrc
done

echo "✅ PC (WSL2) setup done. You can now run: source ~/.bashrc"

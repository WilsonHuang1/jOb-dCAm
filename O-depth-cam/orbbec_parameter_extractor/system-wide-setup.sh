# Copy OrbbecSDK to a standard location and set up environment
sudo cp -r /opt/OrbbecSDK_v2.4.8/include/* /usr/local/include/
sudo cp -r /opt/OrbbecSDK_v2.4.8/lib/* /usr/local/lib/
sudo ldconfig

# Create pkgconfig directory if it doesn't exist
sudo mkdir -p /usr/local/lib/pkgconfig

# Create a pkg-config file
sudo tee /usr/local/lib/pkgconfig/OrbbecSDK.pc > /dev/null << EOF
prefix=/usr/local
exec_prefix=\${prefix}
libdir=\${exec_prefix}/lib
includedir=\${prefix}/include

Name: OrbbecSDK
Description: Orbbec 3D Camera SDK
Version: 2.4.8
Libs: -L\${libdir} -lOrbbecSDK
Cflags: -I\${includedir}
EOF

# Update library cache
sudo ldconfig

echo "OrbbecSDK installed system-wide successfully!"
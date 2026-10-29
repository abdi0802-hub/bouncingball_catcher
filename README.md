# Bouncing Ball Catcher

Package ROS2 berbasis C++ dan OpenCV untuk game interaktif *Bouncing Ball Catcher*. Projek ini mendeteksi objek fisik (tutup botol Aqua) secara real-time menggunakan segmentasi warna HSV untuk menangkap bola virtual pada layar.

Muhammad Abdillah/5025261012

## 1. Arsitektur Singkat (Node, Topic)

### Node & Topic
- **`camera_node`**: Membaca frame dari webcam, membalikkan tampilan secara horizontal (`cv::flip`) agar tidak *mirror*, dan mempublikasikan gambar ke topik `/camera/image_raw`.
- **`game_node`**: Berlangganan ke topik `/camera/image_raw`, memproses segmentasi HSV objek, mengupdate posisi bola, mengelola timer *hold* (2 detik), serta mempublikasikan skor ke `/game/score` dan tampilan visual ke `/game/debug_image`.

## 2. Cara Menjalankan
  
1.**Build Package**:
     ```bash
     colcon build --packages-select bouncingball_catcher
     source install/setup.bash
2. **Jalankan Launch File**
  - ros2 launch bouncingball_catcher game_launch.xml

## 3. Asumsi dan Keputusan
1.Render Langsung via OpenCV: Menggunakan cv::imshow langsung dari node C++ untuk menghindari lag/stuttering akibat overhead konversi pada rqt_image_view.
2.Pembersihan Noise HSV: Mengaplikasikan operasi morfologi (cv::erode & cv::dilate) agar deteksi objek lebih stabil terhadap pencahayaan ruangan.

## 4. Kendala dan solusi
1.Tampilan Kamera Mirror: Tampilan awal webcam terbalik horizontal. Diselesaikan dengan menambahkan fungsi cv::flip(frame, frame, 1) pada camera_node.cpp.
2.Visualisasi GUI Lagging: Penggunaan GUI eksternal memicu drop FPS. Diselesaikan dengan merender langsung dari thread utama menggunakan fungsi bawaan OpenCV.

## 5. DOKUMENTASI DAN ASET
https://drive.google.com/drive/folders/1sIyf-EeanThPAZLghFgO2yimsibZjbwo?usp=sharing


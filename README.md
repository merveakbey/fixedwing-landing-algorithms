# 🛩️ Fixed-Wing Landing Algorithms ROS 2 & PX4

Bu depo, sabit kanatlı UAV sistemleri için otonom uçuş ve seviye iniş algoritmalarını içermektedir. ROS 2 Humble mimarisi ve PX4 SITL Gazebo ortamı kullanılarak geliştirilmiştir.

### 📂 Dosya Yapısı

* **`waypoints_landing.py`**: Uçağı belirli waypoint koordinatları üzerinden matematiksel bir Glide Slope hattına sokarak piste indiren temel yaklaşım algoritması.
* **`attitude_landing.py`**: Uçağın Touchdown anındaki Porpoising ve devrilme sorunlarını kökten çözen algoritma. Otopilotun konum filtreleri aşılarak uçağa doğrudan Pitch, Roll ve Yaw komutları gönderilir.
* **`dynamic_landing.py`**: Otonom uçuş esnasında keyboard interrupt gibi dışarıdan bir tetikleme ile uçağın anlık Yaw açısını kilitleyerek bulunduğu çizgi üzerinden dinamik iniş sekansını başlatan ROS 2 node betiği.

## 🛠 Kullanılan Teknolojiler
* **İşletim Sistemi:** Ubuntu 22.04
* **Middleware:** ROS 2 Humble
* **Otopilot Stack:** PX4 Autopilot 
* **Haberleşme:** MAVROS 
* **Dil:** Python rclpy

## ⚙️ Nasıl Çalıştırılır?

Projeyi simülasyon ortamında test etmek için 3 farklı terminal sekmesinde sırasıyla şu adımları izleyin:

**1. Simülasyonu Başlatın:**

Gazebo ortamında sabit kanatlı aracı (plane) ayağa kaldırmak için PX4 dizininde şu komutu çalıştırın:
```bash
make px4_sitl gazebo_plane
```

2. Haberleşme Köprüsünü (MAVROS) Açın:
Sistemin PX4 ile ROS 2 arasında konuşabilmesi için MAVROS düğümünü başlatın:
Bash
```bash
ros2 launch mavros px4.launch
```

3. İniş Algoritmasını Çalıştırın:
Gilirsiniz:
```bash
python3 attitude_landing.py
```

*Bu proje, otonom hava araçları kontrol sistemleri üzerine yapılan robotik stajı kapsamında geliştirilmiştir.*

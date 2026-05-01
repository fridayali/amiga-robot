# Amiga Robot — ROS2 Docker Setup

Ubuntu 24 + ROS2 Jazzy + Nvidia Xavier (ARM64)

## Repo Yapısı

```
amiga-robot/
├── Dockerfile                        # Xavier için image tarifi
├── entrypoint.sh                     # Container başlangıç scripti
├── docker-compose.yml                # Çalıştırma konfigürasyonu
├── requirements.txt                  # pip paketleri (farm-ng-amiga)
├── manifests/
│   └── amiga_ros2_bridge/
│       └── package.xml               # rosdep için — sadece bağımlılıklar
└── ros2_ws/
    └── src/                          # ROS2 paketlerin buraya gelir
```

## Kurulum

### 1. Repo'yu Xavier'e klonla

```bash
ssh user@xavier-ip
git clone https://github.com/kullanici/amiga-robot.git
cd amiga-robot
```

### 2. ROS2 paketlerini ekle

```bash
# Kendi paketlerini src/ altına koy
cd ros2_ws/src
git clone https://github.com/farm-ng/amiga_ros2_bridge_ws.git
```

### 3. manifests/ güncelle

Her paketin `package.xml`'ini `manifests/` altına kopyala:

```bash
cp ros2_ws/src/amiga_ros2_bridge/package.xml manifests/amiga_ros2_bridge/
```

### 4. Build

```bash
docker build -t amiga-ros2:latest .
```

### 5. Çalıştır

```bash
docker compose up -d
docker compose logs -f
```

## PC ile Haberleşme

PC ve Xavier aynı ağdaysa, PC'de:

```bash
export ROS_DOMAIN_ID=0
ros2 topic list
```

## Güncelleme

```bash
docker compose exec amiga_robot bash
cd /ros2_ws/src && git pull
cd /ros2_ws && colcon build --symlink-install
exit
docker compose restart
```

## Ortam Değişkenleri

| Değişken | Varsayılan | Açıklama |
|---|---|---|
| `ROS_DOMAIN_ID` | 0 | PC ile aynı olmalı |
| `RMW_IMPLEMENTATION` | rmw_fastrtps_cpp | DDS implementasyonu |
| `NVIDIA_VISIBLE_DEVICES` | all | GPU erişimi |

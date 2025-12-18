# MYNT EYE SDK + ROS 2 Node (Build Sources)

This repository contains:
- **MYNT EYE SDK sources** (for building/installing the native SDK and running sample applications)
- **ROS 2 package (node)** for interacting with MYNT EYE cameras from ROS 2

> **Target OS:** Ubuntu 22.04  
> **Important:** The SDK sources in this repository are **modified** relative to the original MYNT EYE SDK so that everything builds correctly on Ubuntu 22.04. In particular, a **legacy OpenCV (3.4.3)** is required and some build flags were adjusted.  
> **Hardware:** MYNT EYE cameras must be connected to **USB 3.0**.

---

## RU — Инструкция по работе с репозиторием (Ubuntu 22.04)

### Что находится в репозитории

- Исходники **MYNT EYE SDK** (сборка нативного SDK + запуск sample-приложений)
- Пакет(ы) **ROS 2** (нода для работы с камерой в ROS 2)

---

## 1. Требования

- Ubuntu 22.04
- Камера MYNT EYE подключена к **USB 3.0**
- Установленный ROS 2 (например, Humble) и рабочее пространство `colcon`
- Инструменты сборки: `gcc/g++`, `cmake`, `make`, `git`

Рекомендация: держите окружение “чистым” и не допускайте, чтобы CMake случайно подхватил другую версию OpenCV из системы.

---

## 2. Зависимости (рекомендуемый минимум)

```bash
sudo apt update
sudo apt install -y \
  build-essential cmake git pkg-config \
  libgtk-3-dev \
  libavcodec-dev libavformat-dev libswscale-dev \
  libtbb2 libtbb-dev \
  libjpeg-dev libpng-dev libtiff-dev \
  libdc1394-dev
3. Сборка и установка OpenCV 3.4.3 (из исходников) в /opt
Этот шаг устанавливает OpenCV в /opt/opencv-3.4.3 и регистрирует библиотеки через ldconfig.

Из директории исходников OpenCV 3.4.3:

bash
Копировать код
mkdir -p _build
cd _build

cmake .. \
  -DCMAKE_BUILD_TYPE=RELEASE \
  -DCMAKE_INSTALL_PREFIX=/opt/opencv-3.4.3 \
  -DWITH_CUDA=OFF \
  -DBUILD_DOCS=OFF \
  -DBUILD_EXAMPLES=OFF \
  -DBUILD_TESTS=OFF \
  -DBUILD_PERF_TESTS=OFF

make -j"$(nproc)"
sudo make install

echo /opt/opencv-3.4.3/lib | sudo tee /etc/ld.so.conf.d/opencv-3.4.3.conf
sudo ldconfig
Указать OpenCV_DIR
В текущей сессии терминала:

bash
Копировать код
export OpenCV_DIR=/opt/opencv-3.4.3/
Чтобы сохранить для новых терминалов:

bash
Копировать код
echo 'export OpenCV_DIR=/opt/opencv-3.4.3/' >> ~/.bashrc
source ~/.bashrc
4. Сборка и установка MYNT EYE SDK
Перейдите в директорию SDK в этом репозитории:

bash
Копировать код
cd <sdk>
4.1 Required Packages / подготовка окружения
bash
Копировать код
make init
Что делает make init:

подготавливает окружение сборки SDK (как правило: инициализирует/подтягивает зависимости, third-party компоненты, проверяет инструменты сборки).

4.2 Build and install
bash
Копировать код
make install
Что делает make install:

собирает SDK и устанавливает его в систему.

По умолчанию SDK обычно устанавливается в:

/usr/local (библиотеки часто попадают в /usr/local/lib, заголовки — в /usr/local/include)

4.3 Building samples
bash
Копировать код
make samples
Что делает make samples:

собирает примеры (sample applications), поставляемые вместе с SDK.

4.4 Run samples
bash
Копировать код
./samples/_output/bin/camera_with_senior_api


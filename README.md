# Инструкция по сборке MYNT EYE SDK + ROS 2 ноды (Ubuntu 22.04)

## Важные замечания

- Репозиторий содержит **модифицированный** MYNT EYE SDK относительно оригинального, чтобы корректно собираться на **Ubuntu 22.04**. В частности, требуется **OpenCV 3.4.3** и для сборки изменены некоторые флаги сборки.
- Камеру MYNT EYE подключайте **строго к USB 3.0**.

---

## 1) Установка базовых зависимостей

```bash
sudo apt update
sudo apt install -y build-essential cmake git pkg-config
sudo apt install -y libgtk-3-dev libavcodec-dev libavformat-dev libswscale-dev
sudo apt install -y libtbb2 libtbb-dev libjpeg-dev libpng-dev libtiff-dev libdc1394-dev
```

---

## 2) Сборка и установка OpenCV 3.4.3 в `/opt/opencv-3.4.3`

Создайте и перейдите в каталог исходников OpenCV 3.4.3:

```bash
cd ~
git clone https://github.com/opencv/opencv.git
cd opencv
git checkout 3.4.3
```

Создайте директорию сборки и выполните конфигурацию:

```bash
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
```

Соберите и установите:

```bash
make -j"$(nproc)"
sudo make install
```

Добавьте путь библиотек OpenCV в динамический линкер и обновите кэш:

```bash
echo /opt/opencv-3.4.3/lib | sudo tee /etc/ld.so.conf.d/opencv-3.4.3.conf
sudo ldconfig
```

Задайте переменную `OpenCV_DIR` для текущей сессии (важно для сборки SDK):

```bash
export OpenCV_DIR=/opt/opencv-3.4.3/
```

---

## 3) Сборка и установка MYNT EYE SDK

Перейдите в каталог SDK внутри репозитория (замените <repo_root>/):

```bash
cd <repo_root>/MYNT-EYE-S-SDK-master
```

Инициализация окружения сборки (Required Packages):

```bash
make init
```

Сборка и установка SDK:

```bash
make install
```

Примечание: по умолчанию SDK обычно устанавливается в `/usr/local` (например, библиотеки в `/usr/local/lib`).

Сборка примеров:

```bash
make samples
```

Запуск примера для проверки работы SDK и камеры:

```bash
./samples/_output/bin/camera_with_senior_api
```

---

## 4) Сборка ROS 2 ноды в workspace и запуск публикации данных

Предполагается, что вы клонировали ROS 2 workspace (`~/ros2_ws`), и пакет ноды лежит в `src`.
Если пакет лежит внутри репозитория, его нужно либо положить/сослать в `~/ros2_ws/src`, либо собирать workspace там, где он уже расположен.

Перейдите в workspace:

```bash
cd ~/ros2_ws
```

Соберите workspace:

```bash
colcon build
```

Подключите окружение:

```bash
source install/setup.bash
```

Запустите launch-файл ноды (имя пакета/launch как в вашем проекте):

```bash
ros2 launch mynt_eye mynt_chek.launch.py
```

---

## 5) Troubleshooting

### 5.1) `libmynteye.so.2: cannot open shared object file`

Это означает, что динамический линкер не видит библиотеки SDK.

Сначала выполните:

```bash
sudo ldconfig
```

Если не помогло — найдите, где лежит библиотека:

```bash
sudo find /usr/local -name "libmynteye.so*" -print
```

Добавьте реальный путь к библиотекам SDK в конфигурацию `ldconfig` (пример для `/usr/local/lib`):

```bash
echo /usr/local/lib | sudo tee /etc/ld.so.conf.d/mynteye.conf
sudo ldconfig
```

---

### 5.2) OpenCV конфликтует по версии

Если в системе несколько OpenCV, CMake может выбрать неправильную версию. Зафиксируйте:

```bash
export OpenCV_DIR=/opt/opencv-3.4.3/
```

---

### 5.3) Нода запускается, но данных нет

Проверьте по порядку:

- Подключение камеры к **USB 3.0**
- Запускается ли SDK пример:
  ```bash
  ./samples/_output/bin/camera_with_senior_api
  ```
  Если пример не работает — проблему сначала решайте на уровне SDK/драйверов/USB.
- Выполнен ли `sudo ldconfig` после `make install` SDK, и доступна ли `libmynteye.so.2`.
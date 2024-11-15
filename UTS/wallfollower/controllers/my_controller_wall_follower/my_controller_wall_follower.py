from controller import Robot

def wall_following(robot):
    """Robot wall follower"""

    # Time step dan kecepatan maksimum
    time_step = int(robot.getBasicTimeStep())
    max_velocity = 6.28

    # Inisialisasi motor
    roda_kiri = robot.getMotor('left wheel motor')
    roda_kanan = robot.getMotor('right wheel motor')
    
    roda_kiri.setPosition(float('inf'))
    roda_kiri.setVelocity(0.0)
    
    roda_kanan.setPosition(float('inf'))
    roda_kanan.setVelocity(0.0)

    # Inisialisasi sensor jarak
    sensor_jarak = []
    for i in range(8):
        nama_sensor = 'ps' + str(i)
        sensor_jarak.append(robot.getDistanceSensor(nama_sensor))
        sensor_jarak[i].enable(time_step)
    
    # Loop kontrol
    while robot.step(time_step) != -1:
        # Membaca data dari sensor
        for i, sensor in enumerate(sensor_jarak):
            print(f"Sensor {i}: Nilai = {sensor.getValue()}")
        
        # Logika deteksi tembok
        tembok_kiri = sensor_jarak[5].getValue() > 80
        sudut_kiri = sensor_jarak[6].getValue() > 80
        tembok_depan = sensor_jarak[7].getValue() > 80

        # Set kecepatan motor default
        kecepatan_kiri = max_velocity
        kecepatan_kanan = max_velocity

        if tembok_depan:
            print("Ada tembok di depan, belok kanan")
            kecepatan_kiri = max_velocity
            kecepatan_kanan = -max_velocity
        else:
            if tembok_kiri:
                print("Jalan lurus, tembok di sebelah kiri")
                kecepatan_kiri = max_velocity
                kecepatan_kanan = max_velocity
            else:
                print("Belok kiri, tidak ada tembok di kiri")
                kecepatan_kiri = max_velocity / 8
                kecepatan_kanan = max_velocity

            if sudut_kiri:
                print("Terlalu dekat ke tembok, sesuaikan ke kanan")
                kecepatan_kiri = max_velocity
                kecepatan_kanan = max_velocity / 8

        # Mengirim perintah ke motor dengan kecepatan yang sudah disesuaikan
        roda_kiri.setVelocity(kecepatan_kiri)
        roda_kanan.setVelocity(kecepatan_kanan)

if __name__ == "__main__":
    # Membuat instance robot dan menjalankan fungsi pengikut tembok
    robot_epuck = Robot()
    wall_following(robot_epuck)

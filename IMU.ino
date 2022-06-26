#include<ros.h>
#include<ros/time.h>
#include<sensor_msgs/Imu.h>

#include <MPU6050_tockn.h>
#include <Wire.h>

MPU6050 mpu6050(Wire);

ros::NodeHandle nh;

sensor_msgs::Imu imu_msg;

ros::Publisher pub("/imu/data_raw", &imu_msg);

char IMU_FRAME[] = "/imu_frame";

//long timer = 0;

float accel_x, accel_y, accel_z;
float gyro_x, gyro_y, gyro_z;

void setup() {
  nh.initNode();
  nh.advertise(pub);
  
 // Serial.begin(9600);
  Wire.begin();
  mpu6050.begin();
  //mpu6050.calcGyroOffsets(true);
}

void loop() {
  mpu6050.update();

  accel_x = mpu6050.getAccX();
  accel_y = mpu6050.getAccY();
  accel_z = mpu6050.getAccZ();

  gyro_x = mpu6050.getGyroX();
  gyro_y = mpu6050.getGyroY();
  gyro_z = mpu6050.getGyroZ();


  imu_msg.header.frame_id = IMU_FRAME;
  imu_msg.linear_acceleration.x = accel_x * 9.8;
  imu_msg.linear_acceleration.y = accel_y * 9.8;
  imu_msg.linear_acceleration.z = accel_z * 9.8;

  imu_msg.angular_velocity.x = gyro_x * 0.0174;
  imu_msg.angular_velocity.y = gyro_y * 0.0174;
  imu_msg.angular_velocity.z = gyro_z * 0.0174;

  imu_msg.header.stamp = nh.now();

  pub.publish(&imu_msg);
  nh.spinOnce();

  //if(millis() - timer > 1000){
//    
//    Serial.println("=======================================================");
//    Serial.print("temp : ");Serial.println(mpu6050.getTemp());
//    Serial.print("accX : ");Serial.print(mpu6050.getAccX());
//    Serial.print("\taccY : ");Serial.print(mpu6050.getAccY());
//    Serial.print("\taccZ : ");Serial.println(mpu6050.getAccZ());
//  
//    Serial.print("gyroX : ");Serial.print(mpu6050.getGyroX());
//    Serial.print("\tgyroY : ");Serial.print(mpu6050.getGyroY());
//    Serial.print("\tgyroZ : ");Serial.println(mpu6050.getGyroZ());
//  
//    Serial.print("accAngleX : ");Serial.print(mpu6050.getAccAngleX());
//    Serial.print("\taccAngleY : ");Serial.println(mpu6050.getAccAngleY());
//  
//    Serial.print("gyroAngleX : ");Serial.print(mpu6050.getGyroAngleX());
//    Serial.print("\tgyroAngleY : ");Serial.print(mpu6050.getGyroAngleY());
//    Serial.print("\tgyroAngleZ : ");Serial.println(mpu6050.getGyroAngleZ());
//    
//    Serial.print("angleX : ");Serial.print(mpu6050.getAngleX());
//    Serial.print("\tangleY : ");Serial.print(mpu6050.getAngleY());
//    Serial.print("\tangleZ : ");Serial.println(mpu6050.getAngleZ());
//    Serial.println("=======================================================\n");
   // timer = millis();
    
  //}

}

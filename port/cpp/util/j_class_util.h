#include "../../../src/util/sensor/ImuSensor.h"
#include "../../../src/util/sensor/LidarSensor.h"
#include "../../../src/util/sensor/Odom.h"
#include "jni_utils.h"
#include <jni.h>
#include <string>
#include <vector>

inline ImuSensor parseImuSensor(JNIEnv *env, jobject imuSensor) {
  jclass imuSensorClass = env->FindClass("port/pwrup/google/potato/ImuSensor");
  jfieldID nameField =
      env->GetFieldID(imuSensorClass, "name", "Ljava/lang/String;");
  jstring imuNameJString = (jstring)env->GetObjectField(imuSensor, nameField);
  std::string imuNameStr = jstringToString(env, imuNameJString);

  env->DeleteLocalRef(imuNameJString);
  return ImuSensor(imuNameStr);
}

inline Odom parseOdom(JNIEnv *env, jobject odomObj) {
  jclass odomClass = env->GetObjectClass(odomObj);

  jfieldID nameField = env->GetFieldID(odomClass, "name", "Ljava/lang/String;");
  jstring javaName = (jstring)env->GetObjectField(odomObj, nameField);
  std::string name = jstringToString(env, javaName);

  return Odom(name);
}

inline LidarSensor parseLidarSensor(JNIEnv *env, jobject lidarSensorObj) {
  jclass lidarSensorClass = env->GetObjectClass(lidarSensorObj);

  jfieldID xField = env->GetFieldID(lidarSensorClass, "x", "F");
  jfieldID yField = env->GetFieldID(lidarSensorClass, "y", "F");
  jfieldID zField = env->GetFieldID(lidarSensorClass, "z", "F");
  jfieldID lidarScanTimeHzField =
      env->GetFieldID(lidarSensorClass, "lidarScanTimeHz", "F");
  jfieldID nameField =
      env->GetFieldID(lidarSensorClass, "name", "Ljava/lang/String;");

  float x = env->GetFloatField(lidarSensorObj, xField);
  float y = env->GetFloatField(lidarSensorObj, yField);
  float z = env->GetFloatField(lidarSensorObj, zField);
  float lidarScanTimeHz =
      env->GetFloatField(lidarSensorObj, lidarScanTimeHzField);
  jstring javaName = (jstring)env->GetObjectField(lidarSensorObj, nameField);
  std::string name = jstringToString(env, javaName);

  return LidarSensor(x, y, z, lidarScanTimeHz, name);
}

inline std::vector<ImuSensor> parseImuSensors(JNIEnv *env, jobject imuSensors) {
  std::vector<ImuSensor> imuSensorsCpp;

  jclass imuSensorClass = env->FindClass("port/pwrup/google/potato/ImuSensor");

  jfieldID nameField =
      env->GetFieldID(imuSensorClass, "name", "Ljava/lang/String;");

  jclass listClass = env->FindClass("java/util/List");
  jmethodID listSizeMethod = env->GetMethodID(listClass, "size", "()I");
  jmethodID listGetMethod =
      env->GetMethodID(listClass, "get", "(I)Ljava/lang/Object;");

  jint imuSensorsSize = env->CallIntMethod(imuSensors, listSizeMethod);

  for (jint i = 0; i < imuSensorsSize; i++) {
    jobject imuSensorObject =
        env->CallObjectMethod(imuSensors, listGetMethod, i);

    imuSensorsCpp.push_back(ImuSensor(parseImuSensor(env, imuSensorObject)));

    env->DeleteLocalRef(imuSensorObject);
  }

  return imuSensorsCpp;
}

inline std::vector<LidarSensor> parseLidarSensorList(JNIEnv *env,
                                                     jobject lidarSensorList) {
  std::vector<LidarSensor> lidarSensors;

  jclass arrayListClass = env->GetObjectClass(lidarSensorList);

  jmethodID sizeMethod = env->GetMethodID(arrayListClass, "size", "()I");

  jmethodID getMethod =
      env->GetMethodID(arrayListClass, "get", "(I)Ljava/lang/Object;");

  jint size = env->CallIntMethod(lidarSensorList, sizeMethod);

  for (jint i = 0; i < size; i++) {
    jobject lidarSensorObj =
        env->CallObjectMethod(lidarSensorList, getMethod, i);
    LidarSensor sensor = parseLidarSensor(env, lidarSensorObj);
    lidarSensors.push_back(sensor);

    env->DeleteLocalRef(lidarSensorObj);
  }

  return lidarSensors;
}

inline std::vector<Odom> parseOdomList(JNIEnv *env, jobject odomList) {
  std::vector<Odom> odoms;

  jclass arrayListClass = env->GetObjectClass(odomList);

  jmethodID sizeMethod = env->GetMethodID(arrayListClass, "size", "()I");

  jmethodID getMethod =
      env->GetMethodID(arrayListClass, "get", "(I)Ljava/lang/Object;");

  jint size = env->CallIntMethod(odomList, sizeMethod);

  for (jint i = 0; i < size; i++) {
    jobject odomObj = env->CallObjectMethod(odomList, getMethod, i);
    Odom odom = parseOdom(env, odomObj);
    odoms.push_back(odom);

    env->DeleteLocalRef(odomObj);
  }

  return odoms;
}
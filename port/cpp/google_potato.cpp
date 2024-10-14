#include "../../src/google_potato.h"
#include "util/j_class_util.h"
#include <jni.h>
#include <string>

class GooglePotatoJava {
public:
  GooglePotatoJava(std::string configDir, std::string mainConfigFile,
                   std::vector<LidarSensor> lidars, std::vector<Odom> odom,
                   std::vector<ImuSensor> imuSensors) {
    potatoInstance = new GooglePotato(
        configDir, mainConfigFile,
        [this](const Position position) {
          if (position.timestamp > lastPosition.timestamp) {
            lastPosition = position;
          }
        },
        lidars, odom, imuSensors);
  }

  void addPointCloudData(PointCloud data) {
    potatoInstance->handleLidarData(data);
  }

  void addImuData(ImuData data) { potatoInstance->handleImuData(data); }

  void addOdomData(OdomData data) { potatoInstance->handleOdomData(data); }

  void stop() { potatoInstance->stopAndOptimize(); }

  Position getLatestPosition() { return lastPosition; }

private:
  Position lastPosition;
  GooglePotato *potatoInstance;
};

extern "C" {
JNIEXPORT void JNICALL Java_port_pwrup_google_potato_GooglePotato_init(
    JNIEnv *env, jobject thisobject, jstring dir, jstring file,
    jobject imuSensors, jobject odom, jobject lidarSensors) {
  const std::string configDirStr = jstringToString(env, dir);
  const std::string configFileStr = jstringToString(env, file);

  const auto odomList = parseOdomList(env, odom);
  const auto imuList = parseImuSensors(env, imuSensors);
  const auto lidarList = parseLidarSensorList(env, lidarSensors);

  const auto potato = new GooglePotatoJava(configDirStr, configFileStr,
                                           lidarList, odomList, imuList);
  ptr_to_obj(env, thisobject, potato);
}

JNIEXPORT void JNICALL Java_port_pwrup_google_potato_GooglePotato_addLidarData(
    JNIEnv *env, jobject thisobject, jlong timestamp, jstring name,
    jfloatArray xVals, jfloatArray yVals, jfloatArray zVals,
    jfloatArray intencities) {
  const auto potato = (GooglePotatoJava *)ptr_from_obj(env, thisobject);
  PointCloud data(SensorIdentity(timestamp, jstringToString(env, name)));
  jsize length = env->GetArrayLength(xVals);

  jfloat *pointsXElements = env->GetFloatArrayElements(xVals, 0);
  jfloat *pointsYElements = env->GetFloatArrayElements(yVals, 0);
  jfloat *pointsZElements = env->GetFloatArrayElements(zVals, 0);
  jfloat *intencitiesElements = env->GetFloatArrayElements(intencities, 0);

  for (int i = 0; i < length; i++) {
    data.add_point(pointsXElements[i], pointsYElements[i], pointsZElements[i],
                   intencitiesElements[i]);
  }

  env->ReleaseFloatArrayElements(xVals, pointsXElements, 0);
  env->ReleaseFloatArrayElements(yVals, pointsYElements, 0);
  env->ReleaseFloatArrayElements(zVals, pointsZElements, 0);
  env->ReleaseFloatArrayElements(intencities, intencitiesElements, 0);

  potato->addPointCloudData(data);
}

JNIEXPORT void JNICALL Java_port_pwrup_google_potato_GooglePotato_addIMUData(
    JNIEnv *env, jobject thisobject, jlong time, jstring name,
    jfloatArray linear, jfloatArray angular) {
  const auto potato = (GooglePotatoJava *)ptr_from_obj(env, thisobject);

  jfloat *linearElements = env->GetFloatArrayElements(linear, 0);
  jsize linearLength = env->GetArrayLength(linear);
  float linearArray[3];
  for (int i = 0; i < linearLength; i++) {
    linearArray[i] = linearElements[i];
  }

  jfloat *angularElements = env->GetFloatArrayElements(angular, 0);
  jsize angularLength = env->GetArrayLength(angular);
  float angularArray[3];
  for (int i = 0; i < angularLength; i++) {
    angularArray[i] = angularElements[i];
  }

  float quaternion[4] = {0, 0, 0, 1};

  ImuData data(SensorIdentity(time, jstringToString(env, name)), linearArray,
               angularArray, quaternion);
  potato->addImuData(data);
}

JNIEXPORT void JNICALL JJava_port_pwrup_google_potato_GooglePotato_addOdomData(
    JNIEnv *env, jobject thisobject, jlong time, jstring name,
    jfloatArray position, jfloatArray quaternion) {
  const auto potato = (GooglePotatoJava *)ptr_from_obj(env, thisobject);
  jfloat *positionElements = env->GetFloatArrayElements(position, 0);
  jsize positionLength = env->GetArrayLength(position);
  float positionArray[3];
  for (int i = 0; i < positionLength; i++) {
    positionArray[i] = positionElements[i];
  }

  jfloat *quaternionElements = env->GetFloatArrayElements(quaternion, 0);
  jsize quaternionLength = env->GetArrayLength(quaternion);
  float quaternionArray[4];
  for (int i = 0; i < quaternionLength; i++) {
    quaternionArray[i] = quaternionElements[i];
  }

  OdomData data(SensorIdentity(time, jstringToString(env, name)),
                positionArray[0], positionArray[1], positionArray[2],
                quaternionArray);

  potato->addOdomData(data);
}

JNIEXPORT void JNICALL
Java_port_pwrup_google_potato_GooglePotato_stopAndOptimize(JNIEnv *env,
                                                           jobject thisobject) {
  const auto potato = (GooglePotato *)ptr_from_obj(env, thisobject);
  potato->stopAndOptimize();
}

JNIEXPORT jfloatArray JNICALL
Java_port_pwrup_google_potato_GooglePotato_getPosition(JNIEnv *env,
                                                       jobject thisobject) {
  const auto potato = (GooglePotatoJava *)ptr_from_obj(env, thisobject);
  auto position = potato->getLatestPosition();
  float arr[6] = {
      static_cast<float>(position.x),     static_cast<float>(position.y),
      static_cast<float>(position.z),     static_cast<float>(position.yaw),
      static_cast<float>(position.pitch), static_cast<float>(position.roll)};

  jfloatArray jfloat_array = env->NewFloatArray(6);
  env->SetFloatArrayRegion(jfloat_array, 0, 6, arr);
  return jfloat_array;
}
}
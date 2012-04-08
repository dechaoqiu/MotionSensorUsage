package org.reno;

import java.util.List;

import android.app.Activity;
import android.content.Context;
import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;
import android.os.Bundle;
import android.widget.TextView;

public class MotionSensorsActivity extends Activity {
	private SensorManager mSensorManager;
	private Sensor gravitySensor;
	private Sensor accelerometerSensor;
	private Sensor gyroscopeSensor;
	private Sensor linearAccelerometeSensor;
	private Sensor rotationVectorSensor;
	private TextView metaPromt;
	private TextView gravityPromt;
	private TextView accelerometerPromt;
	private TextView linearAccelerometerPromt;
	private TextView gyroscopePromt;
	private TextView rotationVectorPromt;

	/** Called when the activity is first created. */
	@Override
	public void onCreate(Bundle savedInstanceState) {
		super.onCreate(savedInstanceState);
		setContentView(R.layout.main);
		mSensorManager = (SensorManager) getSystemService(Context.SENSOR_SERVICE);
		metaPromt = (TextView) findViewById(R.id.metaPromt);
		// 从传感器管理器中获得全部的传感器列表
		List<Sensor> allSensors = mSensorManager.getSensorList(Sensor.TYPE_ALL);
		// 显示有多少个传感器
		metaPromt.setText("经检测该手机有" + allSensors.size() + "个传感器，他们分别是：\n");
		// 显示每个传感器的具体信息
		for (Sensor s : allSensors) {
			String tempString = "\n" + "  设备名称：" + s.getName() + "\n"
					+ "  设备版本：" + s.getVersion() + "\n" + "  供应商："
					+ s.getVendor() + "\n";
			switch (s.getType()) {
			case Sensor.TYPE_ACCELEROMETER:
				metaPromt.setText(metaPromt.getText().toString() + s.getType()
						+ " 加速度传感器accelerometer" + tempString);
				break;
			case Sensor.TYPE_MAGNETIC_FIELD:
				metaPromt.setText(metaPromt.getText().toString() + s.getType()
						+ " 电磁场传感器magnetic field" + tempString);
				break;
			case Sensor.TYPE_ORIENTATION:
				metaPromt.setText(metaPromt.getText().toString() + s.getType()
						+ " 方向传感器orientation" + tempString);
				break;
			case Sensor.TYPE_GYROSCOPE:
				metaPromt.setText(metaPromt.getText().toString() + s.getType()
						+ " 陀螺仪传感器gyroscope" + tempString);
				break;
			case Sensor.TYPE_LIGHT:
				metaPromt.setText(metaPromt.getText().toString() + s.getType()
						+ " 环境光线传感器light" + tempString);
				break;
			case Sensor.TYPE_PRESSURE:
				metaPromt.setText(metaPromt.getText().toString() + s.getType()
						+ " 压力传感器pressure" + tempString);
				break;
			case Sensor.TYPE_TEMPERATURE:
				metaPromt.setText(metaPromt.getText().toString() + s.getType()
						+ " 温度传感器temperature" + tempString);
				break;
			case Sensor.TYPE_PROXIMITY:
				metaPromt.setText(metaPromt.getText().toString() + s.getType()
						+ " 距离传感器proximity" + tempString);
				break;
			case Sensor.TYPE_GRAVITY:
				metaPromt.setText(metaPromt.getText().toString() + s.getType()
						+ " 重力传感器gravity" + tempString);
				break;
			case Sensor.TYPE_ROTATION_VECTOR:
				metaPromt.setText(metaPromt.getText().toString() + s.getType()
						+ " 旋转向量传感器：RotationVector" + tempString);
				break;
			default:
				metaPromt.setText(metaPromt.getText().toString() + s.getType()
						+ " 未知传感器" + tempString);
				break;
			}
		}
		gravitySensor = mSensorManager.getDefaultSensor(Sensor.TYPE_GRAVITY);
		linearAccelerometeSensor = mSensorManager
				.getDefaultSensor(Sensor.TYPE_LINEAR_ACCELERATION);
		accelerometerSensor = mSensorManager
				.getDefaultSensor(Sensor.TYPE_ACCELEROMETER);
		gyroscopeSensor = mSensorManager
				.getDefaultSensor(Sensor.TYPE_GYROSCOPE);
		rotationVectorSensor = mSensorManager
				.getDefaultSensor(Sensor.TYPE_ROTATION_VECTOR);
		gravityPromt = (TextView) findViewById(R.id.gravityPromt);
		accelerometerPromt = (TextView) findViewById(R.id.accelerometerPromt);
		linearAccelerometerPromt = (TextView) findViewById(R.id.linearAccelerometerPromt);
		gyroscopePromt = (TextView) findViewById(R.id.gyroscopePromt);
		rotationVectorPromt = (TextView) findViewById(R.id.rotationVectorPromt);
		mSensorManager.registerListener(new SensorEventListener() {
			float[] gravity = new float[3];
			float[] linear_acceleration = new float[3];

			@Override
			public void onAccuracyChanged(Sensor sensor, int accuracy) {

			}

			@Override
			public void onSensorChanged(SensorEvent event) {
				// In this example, alpha is calculated as t / (t + dT),
				// where t is the low-pass filter's time-constant and
				// dT is the event delivery rate.

				final float alpha = 0.8f;

				// Isolate the force of gravity with the low-pass filter.
				gravity[0] = alpha * gravity[0] + (1 - alpha) * event.values[0];
				gravity[1] = alpha * gravity[1] + (1 - alpha) * event.values[1];
				gravity[2] = alpha * gravity[2] + (1 - alpha) * event.values[2];

				// Remove the gravity contribution with the high-pass filter.
				linear_acceleration[0] = event.values[0] - gravity[0];
				linear_acceleration[1] = event.values[1] - gravity[1];
				linear_acceleration[2] = event.values[2] - gravity[2];
				gravityPromt.setText("gravityPromt: x="
						+ (int) linear_acceleration[0] + "," + "y="
						+ (int) linear_acceleration[1] + "," + "z="
						+ (int) linear_acceleration[2]);
			}
		}, gravitySensor, SensorManager.SENSOR_DELAY_GAME);
		mSensorManager.registerListener(new SensorEventListener() {
			float[] gravity = new float[3];
			float[] linear_acceleration = new float[3];

			@Override
			public void onAccuracyChanged(Sensor sensor, int accuracy) {

			}

			@Override
			public void onSensorChanged(SensorEvent event) {
				// In this example, alpha is calculated as t / (t + dT),
				// where t is the low-pass filter's time-constant and
				// dT is the event delivery rate.

				final float alpha = 0.8f;

				// Isolate the force of gravity with the low-pass filter.
				gravity[0] = alpha * gravity[0] + (1 - alpha) * event.values[0];
				gravity[1] = alpha * gravity[1] + (1 - alpha) * event.values[1];
				gravity[2] = alpha * gravity[2] + (1 - alpha) * event.values[2];

				// Remove the gravity contribution with the high-pass filter.
				linear_acceleration[0] = event.values[0] - gravity[0];
				linear_acceleration[1] = event.values[1] - gravity[1];
				linear_acceleration[2] = event.values[2] - gravity[2];
				accelerometerPromt.setText("accelerometerPromt: x="
						+ (int) linear_acceleration[0] + "," + "y="
						+ (int) linear_acceleration[1] + "," + "z="
						+ (int) linear_acceleration[2]);
			}
		}, accelerometerSensor, SensorManager.SENSOR_DELAY_GAME);

		mSensorManager.registerListener(new SensorEventListener() {
			@Override
			public void onAccuracyChanged(Sensor sensor, int accuracy) {

			}

			@Override
			public void onSensorChanged(SensorEvent event) {
				linearAccelerometerPromt.setText("linearAccelerometerPromt: x="
						+ (int) event.values[0] + "," + "y="
						+ (int) event.values[1] + "," + "z="
						+ (int) event.values[2]);
			}
		}, linearAccelerometeSensor, SensorManager.SENSOR_DELAY_GAME);
		mSensorManager.registerListener(new SensorEventListener() {
			// Create a constant to convert nanoseconds to seconds.
			private static final float NS2S = 1.0f / 1000000000.0f;
			private final float[] deltaRotationVector = new float[4];
			private float timestamp;

			@Override
			public void onAccuracyChanged(Sensor sensor, int accuracy) {
				// TODO Auto-generated method stub
			}

			@Override
			public void onSensorChanged(SensorEvent event) {
				// This timestep's delta rotation to be multiplied by the
				// current rotation
				// after computing it from the gyro sample data.
				if (timestamp != 0) {
					final float dT = (event.timestamp - timestamp) * NS2S;
					// Axis of the rotation sample, not normalized yet.
					float axisX = event.values[0];
					float axisY = event.values[1];
					float axisZ = event.values[2];
					// Calculate the angular speed of the sample
					float omegaMagnitude = (float) Math.sqrt(axisX * axisX
							+ axisY * axisY + axisZ * axisZ);
					// Integrate around this axis with the angular speed by the
					// timestep
					// in order to get a delta rotation from this sample over
					// the timestep
					// We will convert this axis-angle representation of the
					// delta rotation
					// into a quaternion before turning it into the rotation
					// matrix.
					float thetaOverTwo = omegaMagnitude * dT / 2.0f;
					float sinThetaOverTwo = (float) Math.sin(thetaOverTwo);
					float cosThetaOverTwo = (float) Math.cos(thetaOverTwo);
					deltaRotationVector[0] = sinThetaOverTwo * axisX;
					deltaRotationVector[1] = sinThetaOverTwo * axisY;
					deltaRotationVector[2] = sinThetaOverTwo * axisZ;
					deltaRotationVector[3] = cosThetaOverTwo;
				}
				timestamp = event.timestamp;
				float[] deltaRotationMatrix = new float[9];
				SensorManager.getRotationMatrixFromVector(deltaRotationMatrix,
						deltaRotationVector);
				String promt = "";
				for (int i = 0; i < deltaRotationMatrix.length; i++) {
					promt += deltaRotationMatrix[i] + "\n";
				}
				gyroscopePromt.setText("gyroscopePromt: \n" + promt);
				// User code should concatenate the delta rotation we computed
				// with the current rotation
				// in order to get the updated rotation.
				// rotationCurrent = rotationCurrent * deltaRotationMatrix;
			}
		}, gyroscopeSensor, SensorManager.SENSOR_DELAY_GAME);
		mSensorManager.registerListener(new SensorEventListener() {
			private final float[] deltaRotationVector = new float[4];

			@Override
			public void onAccuracyChanged(Sensor sensor, int accuracy) {

			}

			@Override
			public void onSensorChanged(SensorEvent event) {
				deltaRotationVector[0] = event.values[0];
				deltaRotationVector[1] = event.values[1];
				deltaRotationVector[2] = event.values[2];
				// deltaRotationVector[3] = event.values[3];
				float[] deltaRotationMatrix = new float[9];
				SensorManager.getRotationMatrixFromVector(deltaRotationMatrix,
						deltaRotationVector);
				String promt = "";
				for (int i = 0; i < deltaRotationMatrix.length; i++) {
					promt += deltaRotationMatrix[i] + "\n";
				}
				rotationVectorPromt.setText("rotationVectorPromt: \n" + promt);
			}
		}, rotationVectorSensor, SensorManager.SENSOR_DELAY_GAME);
	}
}
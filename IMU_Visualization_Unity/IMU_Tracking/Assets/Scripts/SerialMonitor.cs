using System;
using UnityEngine;
using System.Collections;
using System.IO.Ports;

public class SerialMonitor : MonoBehaviour {
	SerialPort stream = new SerialPort("COM4", 38400);
	string distance;

	// Use this for initialization
	void Start () {
		try{			
			stream.DataReceived += DataReceivedHandler;
			stream.Open();
			Debug.Log("Serial Port Opened!");
		}
		catch(Exception e){
			Debug.Log("Could not open serial port: " + e.Message);

		}
		stream.ReadTimeout = 50;
		/*foreach(string str in SerialPort.GetPortNames())
		{
			Debug.Log(String.Format(str));
		}*/
	}

	void Update () {
		StartCoroutine ("ReadPort");
	}

	IEnumerator ReadPort() {
		while (stream.BytesToRead > 0) {
			Debug.Log (stream.ReadLine ());
		}

		return null;
	}

	private void DataReceivedHandler(
		object sender,
		SerialDataReceivedEventArgs e)
	{
		SerialPort sp = (SerialPort)sender;
		string distance = sp.ReadLine();
		Debug.Log(distance);
	}

	void OnApplicationQuit() 
	{
		stream.Close();
		Debug.Log ("Serial Port Closed!");
	}

}
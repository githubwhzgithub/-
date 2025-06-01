package com.example.bluetooth

import android.Manifest
import android.bluetooth.BluetoothAdapter
import android.bluetooth.BluetoothDevice
import android.bluetooth.BluetoothSocket
import android.content.BroadcastReceiver
import android.content.Context
import android.content.Intent
import android.content.IntentFilter
import android.content.pm.PackageManager
import android.os.Build
import androidx.core.app.ActivityCompat
import kotlinx.coroutines.*
import kotlinx.coroutines.flow.MutableStateFlow
import kotlinx.coroutines.flow.StateFlow
import java.io.IOException
import java.io.InputStream
import java.io.OutputStream
import java.util.*

class CustomBluetoothManager(private val context: Context) {
    private val bluetoothAdapter: BluetoothAdapter? = BluetoothAdapter.getDefaultAdapter()
    private var bluetoothSocket: BluetoothSocket? = null
    private var inputStream: InputStream? = null
    private var outputStream: OutputStream? = null
    private var isConnected = false
    private var readJob: Job? = null
    
    // HC-05 默认UUID
    private val uuid: UUID = UUID.fromString("00001101-0000-1000-8000-00805F9B34FB")
    
    // 状态流
    private val _connectionState = MutableStateFlow<ConnectionState>(ConnectionState.DISCONNECTED)
    val connectionState: StateFlow<ConnectionState> = _connectionState
    
    private val _receivedData = MutableStateFlow("")
    val receivedData: StateFlow<String> = _receivedData
    
    private val _pairedDevices = MutableStateFlow<List<BluetoothDevice>>(emptyList())
    val pairedDevices: StateFlow<List<BluetoothDevice>> = _pairedDevices
    
    private val _discoveredDevices = MutableStateFlow<List<BluetoothDevice>>(emptyList())
    val discoveredDevices: StateFlow<List<BluetoothDevice>> = _discoveredDevices
    
    private var discoveryReceiver: BroadcastReceiver? = null
    
    sealed class ConnectionState {
        object DISCONNECTED : ConnectionState()
        object CONNECTING : ConnectionState()
        data class CONNECTED(val device: BluetoothDevice) : ConnectionState()
        object FAILED : ConnectionState()
    }
    
    init {
        updatePairedDevices()
    }
    
    /**
     * 检查蓝牙权限
     */
    fun hasBluetoothPermissions(): Boolean {
        return if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.S) {
            ActivityCompat.checkSelfPermission(context, Manifest.permission.BLUETOOTH_CONNECT) == PackageManager.PERMISSION_GRANTED &&
            ActivityCompat.checkSelfPermission(context, Manifest.permission.BLUETOOTH_SCAN) == PackageManager.PERMISSION_GRANTED
        } else {
            ActivityCompat.checkSelfPermission(context, Manifest.permission.BLUETOOTH) == PackageManager.PERMISSION_GRANTED &&
            ActivityCompat.checkSelfPermission(context, Manifest.permission.BLUETOOTH_ADMIN) == PackageManager.PERMISSION_GRANTED &&
            ActivityCompat.checkSelfPermission(context, Manifest.permission.ACCESS_FINE_LOCATION) == PackageManager.PERMISSION_GRANTED
        }
    }
    
    /**
     * 获取需要的权限列表
     */
    fun getRequiredPermissions(): Array<String> {
        return if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.S) {
            arrayOf(
                Manifest.permission.BLUETOOTH_CONNECT,
                Manifest.permission.BLUETOOTH_SCAN
            )
        } else {
            arrayOf(
                Manifest.permission.BLUETOOTH,
                Manifest.permission.BLUETOOTH_ADMIN,
                Manifest.permission.ACCESS_FINE_LOCATION
            )
        }
    }
    
    /**
     * 更新已配对设备列表
     */
    fun updatePairedDevices() {
        if (!hasBluetoothPermissions()) return
        
        bluetoothAdapter?.let { adapter ->
            val devices = adapter.bondedDevices?.toList() ?: emptyList()
            _pairedDevices.value = devices
        }
    }
    
    /**
     * 启动蓝牙设备发现
     */
    fun startDiscovery(): Boolean {
        if (!hasBluetoothPermissions()) {
            return false
        }
        
        if (bluetoothAdapter == null || !bluetoothAdapter.isEnabled) {
            return false
        }
        
        // 如果正在发现，先取消
        if (bluetoothAdapter.isDiscovering) {
            bluetoothAdapter.cancelDiscovery()
        }
        
        // 清空之前发现的设备
        _discoveredDevices.value = emptyList()
        
        // 注册广播接收器
        registerDiscoveryReceiver()
        
        // 开始发现新设备
        return bluetoothAdapter.startDiscovery()
    }
    
    /**
     * 注册设备发现广播接收器
     */
    private fun registerDiscoveryReceiver() {
        if (discoveryReceiver != null) {
            return
        }
        
        discoveryReceiver = object : BroadcastReceiver() {
            override fun onReceive(context: Context?, intent: Intent?) {
                when (intent?.action) {
                    BluetoothDevice.ACTION_FOUND -> {
                        val device: BluetoothDevice? = intent.getParcelableExtra(BluetoothDevice.EXTRA_DEVICE)
                        device?.let {
                            val currentDevices = _discoveredDevices.value.toMutableList()
                            if (!currentDevices.contains(it)) {
                                currentDevices.add(it)
                                _discoveredDevices.value = currentDevices
                            }
                        }
                    }
                    BluetoothAdapter.ACTION_DISCOVERY_FINISHED -> {
                        // 发现完成，可以在这里处理
                    }
                }
            }
        }
        
        val filter = IntentFilter().apply {
            addAction(BluetoothDevice.ACTION_FOUND)
            addAction(BluetoothAdapter.ACTION_DISCOVERY_FINISHED)
        }
        
        context.registerReceiver(discoveryReceiver, filter)
    }
    
    /**
     * 停止设备发现
     */
    fun stopDiscovery() {
        bluetoothAdapter?.cancelDiscovery()
        unregisterDiscoveryReceiver()
    }
    
    /**
     * 注销设备发现广播接收器
     */
    private fun unregisterDiscoveryReceiver() {
        discoveryReceiver?.let {
            try {
                context.unregisterReceiver(it)
            } catch (e: IllegalArgumentException) {
                // 接收器可能已经注销
            }
            discoveryReceiver = null
        }
    }
    
    /**
     * 连接到指定设备
     */
    suspend fun connectToDevice(device: BluetoothDevice): Boolean = withContext(Dispatchers.IO) {
        if (!hasBluetoothPermissions()) {
            _connectionState.value = ConnectionState.FAILED
            return@withContext false
        }
        
        try {
            _connectionState.value = ConnectionState.CONNECTING
            
            // 取消发现以提高连接速度
            bluetoothAdapter?.cancelDiscovery()
            
            // 尝试标准连接方法
            var connected = false
            try {
                bluetoothSocket = device.createRfcommSocketToServiceRecord(uuid)
                bluetoothSocket?.connect()
                connected = true
            } catch (e: IOException) {
                // 标准方法失败，尝试备用方法（适用于HC-05等模块）
                try {
                    val method = device.javaClass.getMethod("createRfcommSocket", Int::class.javaPrimitiveType)
                    bluetoothSocket = method.invoke(device, 1) as BluetoothSocket
                    bluetoothSocket?.connect()
                    connected = true
                } catch (e2: Exception) {
                    e2.printStackTrace()
                    throw e // 抛出原始异常
                }
            }
            
            if (connected) {
                // 获取输入输出流
                inputStream = bluetoothSocket?.inputStream
                outputStream = bluetoothSocket?.outputStream
                
                isConnected = true
                _connectionState.value = ConnectionState.CONNECTED(device)
                
                // 开始监听数据
                startListening()
                
                true
            } else {
                false
            }
        } catch (e: Exception) {
            e.printStackTrace()
            disconnect()
            _connectionState.value = ConnectionState.FAILED
            false
        }
    }
    
    /**
     * 断开连接
     */
    fun disconnect() {
        readJob?.cancel()
        
        try {
            inputStream?.close()
            outputStream?.close()
            bluetoothSocket?.close()
        } catch (e: IOException) {
            e.printStackTrace()
        }
        
        inputStream = null
        outputStream = null
        bluetoothSocket = null
        isConnected = false
        
        _connectionState.value = ConnectionState.DISCONNECTED
    }
    
    /**
     * 开始监听数据
     */
    private fun startListening() {
        readJob = CoroutineScope(Dispatchers.IO).launch {
            val buffer = ByteArray(1024)
            
            while (isConnected && !isActive.not()) {
                try {
                    val bytesRead = inputStream?.read(buffer) ?: 0
                    if (bytesRead > 0) {
                        val receivedMessage = String(buffer, 0, bytesRead)
                        _receivedData.value = receivedMessage.trim()
                    }
                } catch (e: IOException) {
                    if (isConnected) {
                        disconnect()
                    }
                    break
                }
            }
        }
    }
    
    /**
     * 发送命令到STM32
     */
    suspend fun sendCommand(command: String): Boolean = withContext(Dispatchers.IO) {
        if (!isConnected || outputStream == null) {
            return@withContext false
        }
        
        try {
            val message = "$command\r\n"
            outputStream?.write(message.toByteArray())
            outputStream?.flush()
            true
        } catch (e: IOException) {
            e.printStackTrace()
            disconnect()
            false
        }
    }
    
    /**
     * 平衡小车控制命令 - 严格匹配STM32蓝牙协议
     */
    suspend fun startBalance() = sendCommand("START")
    suspend fun stopBalance() = sendCommand("STOP")
    suspend fun moveForward(speed: Float = 0.2f) = sendCommand("FORWARD $speed")
    suspend fun moveBackward(speed: Float = 0.2f) = sendCommand("BACKWARD $speed")
    suspend fun turnLeft() = sendCommand("LEFT")
    suspend fun turnRight() = sendCommand("RIGHT")
    suspend fun getStatus() = sendCommand("STATUS")
    suspend fun resetSystem() = sendCommand("RESET")
    suspend fun setSpeed(speed: Float) = sendCommand("SPEED $speed")
    suspend fun setAngle(angle: Float) = sendCommand("ANGLE $angle")
    suspend fun setPID(kp: Float, ki: Float, kd: Float) = sendCommand("PID $kp $ki $kd")
    
    /**
     * 检查是否连接
     */
    fun isConnected(): Boolean = isConnected
    
    /**
     * 清理资源
     */
    fun cleanup() {
        stopDiscovery()
        disconnect()
    }
}
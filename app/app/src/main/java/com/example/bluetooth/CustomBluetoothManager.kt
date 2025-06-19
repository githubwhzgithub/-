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
import java.util.regex.Pattern

// 视觉状态数据类
data class VisionStatus(
    val mode: String = "OFF",
    val errorX: Float = 0f,
    val errorY: Float = 0f
)

// 数据类用于存储解析后的状态信息
data class RobotStatus(
    val pitch: Float = 0f,
    val roll: Float = 0f,
    val speed: Float = 0f,
    val distance: Float = 0f,
    val enabled: Boolean = false,
    val visionStatus: VisionStatus? = null,
    // val visionMode: String = "OFF",
    // val visionErrorX: Float = 0f,
    // val visionErrorY: Float = 0f,
    val lineDetected: Boolean = false,
    val objectDetected: Boolean = false,
    val yawRate: Float = 0f,
    val targetYawRate: Float = 0f,
    val lastUpdateTime: Long = System.currentTimeMillis()
)

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
    
    // 机器人状态流
    private val _robotStatus = MutableStateFlow(RobotStatus())
    val robotStatus: StateFlow<RobotStatus> = _robotStatus
    
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
            if (hasBluetoothPermissions()) {
                val devices = adapter.bondedDevices?.toList() ?: emptyList()
                _pairedDevices.value = devices
            }
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
        if (hasBluetoothPermissions() && bluetoothAdapter.isDiscovering) {
            bluetoothAdapter.cancelDiscovery()
        }
        
        // 清空之前发现的设备
        _discoveredDevices.value = emptyList()
        
        // 注册广播接收器
        registerDiscoveryReceiver()
        
        // 开始发现新设备
        return if (hasBluetoothPermissions()) {
            bluetoothAdapter.startDiscovery()
        } else {
            false
        }
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
            if (hasBluetoothPermissions()) {
                bluetoothAdapter?.cancelDiscovery()
            }
            
            // 尝试标准连接方法
            var connected = false
            try {
                if (hasBluetoothPermissions()) {
                    bluetoothSocket = device.createRfcommSocketToServiceRecord(uuid)
                    bluetoothSocket?.connect()
                    connected = true
                }
            } catch (e: IOException) {
                // 标准方法失败，尝试备用方法（适用于HC-05等模块）
                try {
                    if (hasBluetoothPermissions()) {
                        val method = device.javaClass.getMethod("createRfcommSocket", Int::class.javaPrimitiveType)
                        bluetoothSocket = method.invoke(device, 1) as BluetoothSocket
                        bluetoothSocket?.connect()
                        connected = true
                    }
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
        
        // 清理消息缓冲区
        synchronized(messageBuffer) {
            messageBuffer.clear()
        }
        
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
    
    // 消息缓冲区，用于处理不完整的消息
    private val messageBuffer = StringBuilder()
    
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
                        val receivedData = String(buffer, 0, bytesRead)
                        
                        // 将接收到的数据添加到缓冲区
                        synchronized(messageBuffer) {
                            messageBuffer.append(receivedData)
                            
                            // 处理缓冲区中的完整消息行
                            processBufferedMessages()
                        }
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
     * 处理缓冲区中的完整消息
     */
    private fun processBufferedMessages() {
        val bufferContent = messageBuffer.toString()
        val lines = bufferContent.split("\n")
        
        // 处理除最后一行外的所有完整行
        for (i in 0 until lines.size - 1) {
            val line = lines[i].trim()
            if (line.isNotEmpty()) {
                _receivedData.value = line
                parseStatusMessage(line)
            }
        }
        
        // 保留最后一行（可能不完整）在缓冲区中
        messageBuffer.clear()
        val lastLine = lines.lastOrNull()
        if (!lastLine.isNullOrEmpty() && !bufferContent.endsWith("\n")) {
            messageBuffer.append(lastLine)
        }
        
        // 防止缓冲区过大
        if (messageBuffer.length > 2048) {
            messageBuffer.clear()
        }
    }
    
    // 解析状态消息（单行处理）
    private fun parseStatusMessage(line: String) {
        try {
            var currentStatus = _robotStatus.value
            
            when {
                line.startsWith("STATUS:") -> {
                    currentStatus = parseBasicStatus(line, currentStatus)
                    _robotStatus.value = currentStatus.copy(lastUpdateTime = System.currentTimeMillis())
                }
                line.startsWith("VISION: MODE") -> {
                    currentStatus = parseK230VisionStatus(line, currentStatus)
                    _robotStatus.value = currentStatus.copy(lastUpdateTime = System.currentTimeMillis())
                }
                line.startsWith("VISION:") -> {
                    currentStatus = parseVisionStatus(line, currentStatus)
                    _robotStatus.value = currentStatus.copy(lastUpdateTime = System.currentTimeMillis())
                }
            }
        } catch (e: Exception) {
            // 解析失败时不更新状态
        }
    }
    
    // 解析基本状态信息
    private fun parseBasicStatus(line: String, currentStatus: RobotStatus): RobotStatus {
        try {
            // STATUS: Pitch=0.12 Roll=-0.05 Speed=150.00 Distance=25.3cm Enabled=1 Vision=LINE YawRate=0.1234 Target_YawRate=0.5678
            val pitchPattern = Pattern.compile("Pitch=([+-]?\\d*\\.?\\d+)")
            val rollPattern = Pattern.compile("Roll=([+-]?\\d*\\.?\\d+)")
            val speedPattern = Pattern.compile("Speed=([+-]?\\d*\\.?\\d+)")
            val distancePattern = Pattern.compile("Distance=([+-]?\\d*\\.?\\d+)")
            val enabledPattern = Pattern.compile("Enabled=([01])")
            val visionPattern = Pattern.compile("Vision=(\\w+)")
            val yawRatePattern = Pattern.compile("YawRate=([+-]?\\d*\\.?\\d+)")
            val targetYawRatePattern = Pattern.compile("Target_YawRate=([+-]?\\d*\\.?\\d+)")
            
            val pitch = pitchPattern.matcher(line).let { if (it.find()) it.group(1)?.toFloatOrNull() ?: currentStatus.pitch else currentStatus.pitch }
            val roll = rollPattern.matcher(line).let { if (it.find()) it.group(1)?.toFloatOrNull() ?: currentStatus.roll else currentStatus.roll }
            val speed = speedPattern.matcher(line).let { if (it.find()) it.group(1)?.toFloatOrNull() ?: currentStatus.speed else currentStatus.speed }
            val distance = distancePattern.matcher(line).let { if (it.find()) it.group(1)?.toFloatOrNull() ?: currentStatus.distance else currentStatus.distance }
            val enabled = enabledPattern.matcher(line).let { if (it.find()) it.group(1) == "1" else currentStatus.enabled }
            val visionMode = visionPattern.matcher(line).let { if (it.find()) it.group(1) ?: (currentStatus.visionStatus?.mode ?: "OFF") else (currentStatus.visionStatus?.mode ?: "OFF") }
            val yawRate = yawRatePattern.matcher(line).let { if (it.find()) it.group(1)?.toFloatOrNull() ?: currentStatus.yawRate else currentStatus.yawRate }
            val targetYawRate = targetYawRatePattern.matcher(line).let { if (it.find()) it.group(1)?.toFloatOrNull() ?: currentStatus.targetYawRate else currentStatus.targetYawRate }
            
            return currentStatus.copy(
                pitch = pitch,
                roll = roll,
                speed = speed,
                distance = distance,
                enabled = enabled,
                yawRate = yawRate,
                targetYawRate = targetYawRate,
                visionStatus = VisionStatus(
                    mode = visionMode,
                    errorX = currentStatus.visionStatus?.errorX ?: 0f,
                    errorY = currentStatus.visionStatus?.errorY ?: 0f
                )
            )
        } catch (e: Exception) {
            return currentStatus
        }
    }
    
    // 解析K230视觉状态信息 (新格式: VISION: MODE %s ErrorX=%.3f ErrorY=%.3f)
    private fun parseK230VisionStatus(line: String, currentStatus: RobotStatus): RobotStatus {
        try {
            // VISION: MODE LINE ErrorX=0.123 ErrorY=-0.045
            val k230Pattern = Pattern.compile("VISION: MODE (\\w+) ErrorX=([+-]?\\d*\\.?\\d+) ErrorY=([+-]?\\d*\\.?\\d+)")
            val matcher = k230Pattern.matcher(line)
            
            if (matcher.find()) {
                val mode = matcher.group(1) ?: "OFF"
                val errorX = matcher.group(2)?.toFloatOrNull() ?: 0f
                val errorY = matcher.group(3)?.toFloatOrNull() ?: 0f
                
                return currentStatus.copy(
                    visionStatus = VisionStatus(
                        mode = mode,
                        errorX = errorX,
                        errorY = errorY
                    )
                )
            }
            
            return currentStatus
        } catch (e: Exception) {
            return currentStatus
        }
    }
    
    // 解析视觉状态信息 (旧格式)
    private fun parseVisionStatus(line: String, currentStatus: RobotStatus): RobotStatus {
        try {
            // VISION: ErrorX=0.123 ErrorY=-0.045 LineDetected=1 ObjectDetected=0
            val errorXPattern = Pattern.compile("ErrorX=([+-]?\\d*\\.?\\d+)")
            val errorYPattern = Pattern.compile("ErrorY=([+-]?\\d*\\.?\\d+)")
            val lineDetectedPattern = Pattern.compile("LineDetected=([01])")
            val objectDetectedPattern = Pattern.compile("ObjectDetected=([01])")
            
            val errorX = errorXPattern.matcher(line).let { if (it.find()) it.group(1)?.toFloatOrNull() ?: (currentStatus.visionStatus?.errorX ?: 0f) else (currentStatus.visionStatus?.errorX ?: 0f) }
            val errorY = errorYPattern.matcher(line).let { if (it.find()) it.group(1)?.toFloatOrNull() ?: (currentStatus.visionStatus?.errorY ?: 0f) else (currentStatus.visionStatus?.errorY ?: 0f) }
            val lineDetected = lineDetectedPattern.matcher(line).let { if (it.find()) it.group(1) == "1" else currentStatus.lineDetected }
            val objectDetected = objectDetectedPattern.matcher(line).let { if (it.find()) it.group(1) == "1" else currentStatus.objectDetected }
            
            return currentStatus.copy(
                visionStatus = VisionStatus(
                    mode = currentStatus.visionStatus?.mode ?: "OFF",
                    errorX = errorX,
                    errorY = errorY
                ),
                lineDetected = lineDetected,
                objectDetected = objectDetected
            )
        } catch (e: Exception) {
            return currentStatus
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
    suspend fun moveForward(speed: Float = 0.05f) = sendCommand("FORWARD $speed")
    suspend fun moveBackward(speed: Float = 0.05f) = sendCommand("BACKWARD $speed")
    suspend fun turnLeft() = sendCommand("LEFT")
    suspend fun turnRight() = sendCommand("RIGHT")
    suspend fun getStatus() = sendCommand("STATUS")
    suspend fun resetSystem() = sendCommand("RESET")
    suspend fun setSpeed(speed: Float) = sendCommand("SPEED $speed")
    suspend fun setAngle(angle: Float) = sendCommand("ANGLE $angle")
    suspend fun setPID(kp: Float, ki: Float, kd: Float) = sendCommand("PID $kp $ki $kd")
    
    /**
     * 视觉模式控制命令 - 严格匹配STM32蓝牙协议
     */
    suspend fun setVisionMode(mode: Int) = sendCommand("VISION $mode")
    suspend fun enableLineTracking() = sendCommand("LINE")
    suspend fun enableObjectTracking() = sendCommand("TRACK")
    suspend fun disableVision() = sendCommand("VOFF")
    
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
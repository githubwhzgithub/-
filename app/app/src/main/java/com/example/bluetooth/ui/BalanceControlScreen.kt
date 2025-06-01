package com.example.bluetooth.ui

import android.annotation.SuppressLint
import android.bluetooth.BluetoothDevice
import androidx.compose.foundation.background
import androidx.compose.foundation.clickable
import androidx.compose.foundation.layout.*
import androidx.compose.foundation.lazy.LazyColumn
import androidx.compose.foundation.lazy.items
import androidx.compose.foundation.shape.CircleShape
import androidx.compose.foundation.shape.RoundedCornerShape
import androidx.compose.material.icons.Icons
import androidx.compose.material.icons.filled.Add
import androidx.compose.material.icons.filled.Bluetooth
import androidx.compose.material.icons.filled.BluetoothConnected
import androidx.compose.material.icons.filled.BluetoothDisabled
import androidx.compose.material3.*
import androidx.compose.runtime.*
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.draw.clip
import androidx.compose.ui.draw.shadow
import androidx.compose.ui.graphics.Brush
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.text.font.FontWeight
import androidx.compose.ui.text.style.TextAlign
import androidx.compose.ui.unit.dp
import androidx.compose.ui.unit.sp
import com.example.bluetooth.CustomBluetoothManager
import kotlinx.coroutines.launch

@Composable
fun BalanceControlScreen(
    bluetoothManager: CustomBluetoothManager,
    modifier: Modifier = Modifier
) {
    var showControlPanel by remember { mutableStateOf(false) }
    var connectedDevice by remember { mutableStateOf<BluetoothDevice?>(null) }
    val connectionState by bluetoothManager.connectionState.collectAsState()
    val receivedData by bluetoothManager.receivedData.collectAsState()
    val pairedDevices by bluetoothManager.pairedDevices.collectAsState()
    val discoveredDevices by bluetoothManager.discoveredDevices.collectAsState()
    var selectedDevice by remember { mutableStateOf<BluetoothDevice?>(null) }
    val scope = rememberCoroutineScope()
    
    // 监听连接状态变化
    LaunchedEffect(connectionState) {
        val currentConnectionState = connectionState
        when (currentConnectionState) {
            is CustomBluetoothManager.ConnectionState.CONNECTED -> {
                connectedDevice = currentConnectionState.device
                showControlPanel = true
            }
            else -> {
                showControlPanel = false
                connectedDevice = null
            }
        }
    }
    
    // 根据连接状态显示不同界面
    if (showControlPanel && connectedDevice != null) {
        ControlPanelScreen(
            bluetoothManager = bluetoothManager,
            connectedDevice = connectedDevice,
            onBackPressed = {
                scope.launch {
                    bluetoothManager.disconnect()
                }
            }
        )
    } else {
        Box(
            modifier = Modifier
                .fillMaxSize()
                .background(
                    Brush.verticalGradient(
                        colors = listOf(
                            Color(0xFF1e3c72),
                            Color(0xFF2a5298)
                        )
                    )
                )
        ) {
            // 背景装饰圆圈
            Box(
                modifier = Modifier
                    .size(300.dp)
                    .offset(x = 200.dp, y = (-100).dp)
                    .background(
                        Color.White.copy(alpha = 0.05f),
                        CircleShape
                    )
            )
            
            LazyColumn(
                modifier = Modifier
                    .fillMaxSize()
                    .padding(16.dp),
                verticalArrangement = Arrangement.spacedBy(16.dp)
            ) {
                // 标题
                item {
                    Column(
                        modifier = Modifier.fillMaxWidth(),
                        horizontalAlignment = Alignment.CenterHorizontally
                    ) {
                        Text(
                            text = "Balance Robot",
                            fontSize = 32.sp,
                            fontWeight = FontWeight.Bold,
                            color = Color.White
                        )
                        Text(
                            text = "Bluetooth Controller",
                            fontSize = 16.sp,
                            color = Color.White.copy(alpha = 0.8f)
                        )
                    }
                }
                
                // 最近活跃设备卡片
                item {
                    RecentlyActiveCard(
                        pairedDevices = pairedDevices,
                        discoveredDevices = discoveredDevices,
                        connectionState = connectionState,
                        onDeviceClick = { device ->
                            selectedDevice = device
                            scope.launch {
                                bluetoothManager.connectToDevice(device)
                            }
                        },
                        onDiscoverDevices = {
                            scope.launch {
                                bluetoothManager.startDiscovery()
                            }
                        }
                    )
                }
                
                // 控制按钮区域（仅在连接时显示）
                if (connectionState is CustomBluetoothManager.ConnectionState.CONNECTED) {
                    item {
                        ControlButtonsSection(
                            onCommand = { command ->
                                scope.launch {
                                    bluetoothManager.sendCommand(command)
                                }
                            }
                        )
                    }
                }
                
                // 数据显示区域
                if (receivedData.isNotEmpty()) {
                    item {
                        DataDisplayCard(data = receivedData)
                    }
                }
            }
        }
    }
}

@Composable
fun RecentlyActiveCard(
    pairedDevices: List<BluetoothDevice>,
    discoveredDevices: List<BluetoothDevice>,
    connectionState: CustomBluetoothManager.ConnectionState,
    onDeviceClick: (BluetoothDevice) -> Unit,
    onDiscoverDevices: () -> Unit
) {
    Card(
        modifier = Modifier
            .fillMaxWidth()
            .shadow(12.dp, RoundedCornerShape(20.dp)),
        colors = CardDefaults.cardColors(
            containerColor = Color.White
        ),
        shape = RoundedCornerShape(20.dp)
    ) {
        Column(
            modifier = Modifier.padding(20.dp)
        ) {
            Text(
                text = "Available Devices",
                fontSize = 20.sp,
                fontWeight = FontWeight.Bold,
                color = Color(0xFF2D3748)
            )
            
            Spacer(modifier = Modifier.height(16.dp))
            
            // 配对设备
            if (pairedDevices.isNotEmpty()) {
                Text(
                    text = "Paired Devices",
                    fontSize = 14.sp,
                    fontWeight = FontWeight.Medium,
                    color = Color(0xFF4A5568)
                )
                Spacer(modifier = Modifier.height(8.dp))
                
                pairedDevices.forEach { device ->
                    DeviceItem(
                        device = device,
                        connectionState = connectionState,
                        onClick = { onDeviceClick(device) }
                    )
                }
            }
            
            // 发现的设备
            if (discoveredDevices.isNotEmpty()) {
                Spacer(modifier = Modifier.height(12.dp))
                Text(
                    text = "Discovered Devices",
                    fontSize = 14.sp,
                    fontWeight = FontWeight.Medium,
                    color = Color(0xFF4A5568)
                )
                Spacer(modifier = Modifier.height(8.dp))
                
                discoveredDevices.forEach { device ->
                    DeviceItem(
                        device = device,
                        connectionState = connectionState,
                        onClick = { onDeviceClick(device) }
                    )
                }
            }
            
            Spacer(modifier = Modifier.height(16.dp))
            
            // 添加设备按钮
            AddDeviceButton(onClick = onDiscoverDevices)
        }
    }
}

@Composable
fun DeviceItem(
    device: BluetoothDevice,
    connectionState: CustomBluetoothManager.ConnectionState,
    onClick: () -> Unit
) {
    val isConnected = connectionState is CustomBluetoothManager.ConnectionState.CONNECTED && connectionState.device == device
    val isConnecting = connectionState is CustomBluetoothManager.ConnectionState.CONNECTING
    
    Row(
        modifier = Modifier
            .fillMaxWidth()
            .clip(RoundedCornerShape(12.dp))
            .background(
                if (isConnected) Color(0xFFE6FFFA) else Color(0xFFF7FAFC)
            )
            .clickable { onClick() }
            .padding(12.dp),
        verticalAlignment = Alignment.CenterVertically
    ) {
        Icon(
            imageVector = when (val currentConnectionState = connectionState) { // Use local variable for smart cast
                is CustomBluetoothManager.ConnectionState.CONNECTED -> Icons.Filled.BluetoothConnected
                is CustomBluetoothManager.ConnectionState.CONNECTING -> Icons.Filled.Bluetooth
                else -> Icons.Filled.BluetoothDisabled
            },
            contentDescription = null,
            tint = when (val currentConnectionState = connectionState) { // Use local variable for smart cast
                is CustomBluetoothManager.ConnectionState.CONNECTED -> Color(0xFF38B2AC)
                is CustomBluetoothManager.ConnectionState.CONNECTING -> Color(0xFF3182CE)
                else -> Color(0xFF718096)
            },
            modifier = Modifier.size(24.dp)
        )
        
        Spacer(modifier = Modifier.width(12.dp))
        
        Column(modifier = Modifier.weight(1f)) {
            @SuppressLint("MissingPermission")
            Text(
                text = device.name ?: "Unknown Device",
                fontSize = 16.sp,
                fontWeight = FontWeight.Medium,
                color = Color(0xFF2D3748)
            )
            Text(
                text = device.address,
                fontSize = 12.sp,
                color = Color(0xFF718096)
            )
        }
        
        if (isConnecting) {
            CircularProgressIndicator(
                modifier = Modifier.size(20.dp),
                strokeWidth = 2.dp,
                color = Color(0xFF3182CE)
            )
        }
    }
    
    Spacer(modifier = Modifier.height(8.dp))
}

@Composable
fun AddDeviceButton(onClick: () -> Unit) {
    Row(
        modifier = Modifier
            .fillMaxWidth()
            .clip(RoundedCornerShape(12.dp))
            .background(Color(0xFF667eea))
            .clickable { onClick() }
            .padding(16.dp),
        verticalAlignment = Alignment.CenterVertically,
        horizontalArrangement = Arrangement.Center
    ) {
        Icon(
            imageVector = Icons.Filled.Add,
            contentDescription = "Add Device",
            tint = Color.White,
            modifier = Modifier.size(20.dp)
        )
        Spacer(modifier = Modifier.width(8.dp))
        Text(
            text = "Discover New Devices",
            color = Color.White,
            fontSize = 16.sp,
            fontWeight = FontWeight.Medium
        )
    }
}

@Composable
fun ControlButtonsSection(
    onCommand: (String) -> Unit
) {
    Card(
        modifier = Modifier
            .fillMaxWidth()
            .shadow(12.dp, RoundedCornerShape(20.dp)),
        colors = CardDefaults.cardColors(
            containerColor = Color.White
        ),
        shape = RoundedCornerShape(20.dp)
    ) {
        Column(
            modifier = Modifier.padding(20.dp),
            horizontalAlignment = Alignment.CenterHorizontally
        ) {
            Text(
                text = "Robot Control",
                fontSize = 20.sp,
                fontWeight = FontWeight.Bold,
                color = Color(0xFF2D3748)
            )
            
            Spacer(modifier = Modifier.height(20.dp))
            
            // 轮盘控制区域
            WheelControlPad(onCommand = onCommand)
            
            Spacer(modifier = Modifier.height(20.dp))
            
            // 停止按钮
            Button(
                onClick = { onCommand("S") },
                modifier = Modifier
                    .fillMaxWidth()
                    .height(50.dp),
                colors = ButtonDefaults.buttonColors(
                    containerColor = Color(0xFFE53E3E)
                ),
                shape = RoundedCornerShape(25.dp)
            ) {
                Text(
                    text = "EMERGENCY STOP",
                    color = Color.White,
                    fontSize = 16.sp,
                    fontWeight = FontWeight.Bold
                )
            }
        }
    }
}

@Composable
fun WheelControlPad(
    onCommand: (String) -> Unit
) {
    Box(
        modifier = Modifier.size(200.dp),
        contentAlignment = Alignment.Center
    ) {
        // 外圆环
        Box(
            modifier = Modifier
                .size(200.dp)
                .background(
                    Color(0xFFF7FAFC),
                    CircleShape
                )
        )
        
        // 前进按钮
        Box(
            modifier = Modifier
                .offset(y = (-60).dp)
                .size(50.dp)
        ) {
            Button(
                onClick = { onCommand("F") },
                modifier = Modifier.fillMaxSize(),
                colors = ButtonDefaults.buttonColors(
                    containerColor = Color(0xFF48BB78)
                ),
                shape = CircleShape,
                contentPadding = PaddingValues(0.dp)
            ) {
                Text(
                    text = "F",
                    color = Color.White,
                    fontSize = 20.sp,
                    fontWeight = FontWeight.Bold
                )
            }
        }
        
        // 后退按钮
        Box(
            modifier = Modifier
                .offset(y = 60.dp)
                .size(50.dp)
        ) {
            Button(
                onClick = { onCommand("B") },
                modifier = Modifier.fillMaxSize(),
                colors = ButtonDefaults.buttonColors(
                    containerColor = Color(0xFF3182CE)
                ),
                shape = CircleShape,
                contentPadding = PaddingValues(0.dp)
            ) {
                Text(
                    text = "B",
                    color = Color.White,
                    fontSize = 20.sp,
                    fontWeight = FontWeight.Bold
                )
            }
        }
        
        // 左转按钮
        Box(
            modifier = Modifier
                .offset(x = (-60).dp)
                .size(50.dp)
        ) {
            Button(
                onClick = { onCommand("L") },
                modifier = Modifier.fillMaxSize(),
                colors = ButtonDefaults.buttonColors(
                    containerColor = Color(0xFFED8936)
                ),
                shape = CircleShape,
                contentPadding = PaddingValues(0.dp)
            ) {
                Text(
                    text = "L",
                    color = Color.White,
                    fontSize = 20.sp,
                    fontWeight = FontWeight.Bold
                )
            }
        }
        
        // 右转按钮
        Box(
            modifier = Modifier
                .offset(x = 60.dp)
                .size(50.dp)
        ) {
            Button(
                onClick = { onCommand("R") },
                modifier = Modifier.fillMaxSize(),
                colors = ButtonDefaults.buttonColors(
                    containerColor = Color(0xFF9F7AEA)
                ),
                shape = CircleShape,
                contentPadding = PaddingValues(0.dp)
            ) {
                Text(
                    text = "R",
                    color = Color.White,
                    fontSize = 20.sp,
                    fontWeight = FontWeight.Bold
                )
            }
        }
        
        // 中心点
        Box(
            modifier = Modifier
                .size(20.dp)
                .background(
                    Color(0xFF667eea),
                    CircleShape
                )
        )
    }
}

@Composable
fun DataDisplayCard(data: String) {
    Card(
        modifier = Modifier
            .fillMaxWidth()
            .shadow(8.dp, RoundedCornerShape(20.dp)),
        colors = CardDefaults.cardColors(
            containerColor = Color.White
        ),
        shape = RoundedCornerShape(20.dp)
    ) {
        Column(
            modifier = Modifier.padding(20.dp)
        ) {
            Text(
                text = "Received Data",
                fontSize = 18.sp,
                fontWeight = FontWeight.Bold,
                color = Color(0xFF2D3748)
            )
            
            Spacer(modifier = Modifier.height(12.dp))
            
            Text(
                text = data,
                fontSize = 14.sp,
                color = Color(0xFF4A5568),
                modifier = Modifier
                    .fillMaxWidth()
                    .background(
                        Color(0xFFF7FAFC),
                        RoundedCornerShape(8.dp)
                    )
                    .padding(12.dp)
            )
        }
    }
}
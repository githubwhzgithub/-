package com.example.bluetooth.ui

import android.annotation.SuppressLint
import android.bluetooth.BluetoothDevice
import androidx.compose.foundation.background
import androidx.compose.foundation.layout.*
import androidx.compose.foundation.lazy.LazyColumn
import androidx.compose.foundation.shape.CircleShape
import androidx.compose.foundation.shape.RoundedCornerShape
import androidx.compose.material.icons.Icons
import androidx.compose.material.icons.filled.ArrowBack
import androidx.compose.material.icons.filled.Settings
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

@OptIn(ExperimentalMaterial3Api::class)
@Composable
fun ControlPanelScreen(
    bluetoothManager: CustomBluetoothManager,
    connectedDevice: BluetoothDevice?,
    onBackPressed: () -> Unit,
    modifier: Modifier = Modifier
) {
    val connectionState by bluetoothManager.connectionState.collectAsState()
    val receivedData by bluetoothManager.receivedData.collectAsState()
    val scope = rememberCoroutineScope()
    
    var isBalanceEnabled by remember { mutableStateOf(false) }
    var currentSpeed by remember { mutableStateOf(0.2f) }
    var currentAngle by remember { mutableStateOf(0f) }
    var showPIDDialog by remember { mutableStateOf(false) }
    
    // 如果连接断开，自动返回
    LaunchedEffect(connectionState) {
        if (connectionState !is CustomBluetoothManager.ConnectionState.CONNECTED) {
            onBackPressed()
        }
    }
    
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
        // 背景装饰
        Box(
            modifier = Modifier
                .size(300.dp)
                .offset(x = 200.dp, y = (-100).dp)
                .background(
                    Color.White.copy(alpha = 0.05f),
                    CircleShape
                )
        )
        
        Column(
            modifier = Modifier
                .fillMaxSize()
                .padding(16.dp)
        ) {
            // 顶部栏
            TopAppBar(
                title = {
                    Column {
                        Text(
                            text = "Balance Control Panel",
                            color = Color.White,
                            fontSize = 20.sp,
                            fontWeight = FontWeight.Bold
                        )
                        connectedDevice?.let { device ->
                            @SuppressLint("MissingPermission")
                            Text(
                                text = "Connected to: ${if (bluetoothManager.hasBluetoothPermissions()) device.name ?: "Unknown" else "Unknown"}",
                                color = Color.White.copy(alpha = 0.8f),
                                fontSize = 12.sp
                            )
                        }
                    }
                },
                navigationIcon = {
                    IconButton(onClick = onBackPressed) {
                        Icon(
                            Icons.Default.ArrowBack,
                            contentDescription = "Back",
                            tint = Color.White
                        )
                    }
                },
                actions = {
                    IconButton(onClick = { showPIDDialog = true }) {
                        Icon(
                            Icons.Default.Settings,
                            contentDescription = "PID Settings",
                            tint = Color.White
                        )
                    }
                },
                colors = TopAppBarDefaults.topAppBarColors(
                    containerColor = Color.Transparent
                )
            )
            
            LazyColumn(
                modifier = Modifier.fillMaxSize(),
                verticalArrangement = Arrangement.spacedBy(16.dp)
            ) {
                // 系统状态卡片
                item {
                    SystemStatusCard(
                        isBalanceEnabled = isBalanceEnabled,
                        onToggleBalance = { enabled ->
                            isBalanceEnabled = enabled
                            scope.launch {
                                if (enabled) {
                                    bluetoothManager.startBalance()
                                } else {
                                    bluetoothManager.stopBalance()
                                }
                            }
                        },
                        onReset = {
                            scope.launch {
                                bluetoothManager.resetSystem()
                                isBalanceEnabled = false
                                currentSpeed = 0.2f
                                currentAngle = 0f
                            }
                        },
                        onGetStatus = {
                            scope.launch {
                                bluetoothManager.getStatus()
                            }
                        }
                    )
                }
                
                // 方向控制卡片
                item {
                    DirectionControlCard(
                        onForward = {
                            scope.launch {
                                bluetoothManager.moveForward(currentSpeed)
                            }
                        },
                        onBackward = {
                            scope.launch {
                                bluetoothManager.moveBackward(currentSpeed)
                            }
                        },
                        onLeft = {
                            scope.launch {
                                bluetoothManager.turnLeft()
                            }
                        },
                        onRight = {
                            scope.launch {
                                bluetoothManager.turnRight()
                            }
                        },
                        onStop = {
                            scope.launch {
                                bluetoothManager.stopBalance()
                                isBalanceEnabled = false
                            }
                        }
                    )
                }
                
                // 速度控制卡片
                item {
                    SpeedControlCard(
                        currentSpeed = currentSpeed,
                        onSpeedChange = { speed ->
                            currentSpeed = speed
                            scope.launch {
                                bluetoothManager.setSpeed(speed)
                            }
                        }
                    )
                }
                
                // 角度控制卡片
                item {
                    AngleControlCard(
                        currentAngle = currentAngle,
                        onAngleChange = { angle ->
                            currentAngle = angle
                            scope.launch {
                                bluetoothManager.setAngle(angle)
                            }
                        }
                    )
                }
                
                // 数据显示卡片
                if (receivedData.isNotEmpty()) {
                    item {
                        DataDisplayCard(
                            data = receivedData,
                            title = "Robot Status"
                        )
                    }
                }
            }
        }
    }
    
    // PID设置对话框
    if (showPIDDialog) {
        PIDSettingsDialog(
            onDismiss = { showPIDDialog = false },
            onConfirm = { kp, ki, kd ->
                scope.launch {
                    bluetoothManager.setPID(kp, ki, kd)
                }
                showPIDDialog = false
            }
        )
    }
}

@Composable
fun SystemStatusCard(
    isBalanceEnabled: Boolean,
    onToggleBalance: (Boolean) -> Unit,
    onReset: () -> Unit,
    onGetStatus: () -> Unit
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
            modifier = Modifier.padding(24.dp)
        ) {
            Text(
                text = "System Control",
                fontSize = 20.sp,
                fontWeight = FontWeight.Bold,
                color = Color(0xFF2D3748)
            )
            
            Spacer(modifier = Modifier.height(20.dp))
            
            // 平衡开关
            Row(
                modifier = Modifier.fillMaxWidth(),
                horizontalArrangement = Arrangement.SpaceBetween,
                verticalAlignment = Alignment.CenterVertically
            ) {
                Column {
                    Text(
                        text = "Balance Mode",
                        fontSize = 16.sp,
                        fontWeight = FontWeight.Medium,
                        color = Color(0xFF2D3748)
                    )
                    Text(
                        text = if (isBalanceEnabled) "Enabled" else "Disabled",
                        fontSize = 12.sp,
                        color = if (isBalanceEnabled) Color(0xFF48BB78) else Color(0xFF718096)
                    )
                }
                
                Switch(
                    checked = isBalanceEnabled,
                    onCheckedChange = onToggleBalance,
                    colors = SwitchDefaults.colors(
                        checkedThumbColor = Color.White,
                        checkedTrackColor = Color(0xFF48BB78),
                        uncheckedThumbColor = Color.White,
                        uncheckedTrackColor = Color(0xFFE2E8F0)
                    )
                )
            }
            
            Spacer(modifier = Modifier.height(20.dp))
            
            // 控制按钮
            Row(
                modifier = Modifier.fillMaxWidth(),
                horizontalArrangement = Arrangement.spacedBy(12.dp)
            ) {
                Button(
                    onClick = onReset,
                    modifier = Modifier.weight(1f),
                    colors = ButtonDefaults.buttonColors(
                        containerColor = Color(0xFFE53E3E)
                    ),
                    shape = RoundedCornerShape(12.dp)
                ) {
                    Text(
                        text = "RESET",
                        color = Color.White,
                        fontWeight = FontWeight.Bold
                    )
                }
                
                Button(
                    onClick = onGetStatus,
                    modifier = Modifier.weight(1f),
                    colors = ButtonDefaults.buttonColors(
                        containerColor = Color(0xFF3182CE)
                    ),
                    shape = RoundedCornerShape(12.dp)
                ) {
                    Text(
                        text = "STATUS",
                        color = Color.White,
                        fontWeight = FontWeight.Bold
                    )
                }
            }
        }
    }
}

@Composable
fun DirectionControlCard(
    onForward: () -> Unit,
    onBackward: () -> Unit,
    onLeft: () -> Unit,
    onRight: () -> Unit,
    onStop: () -> Unit
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
            modifier = Modifier.padding(24.dp),
            horizontalAlignment = Alignment.CenterHorizontally
        ) {
            Text(
                text = "Direction Control",
                fontSize = 20.sp,
                fontWeight = FontWeight.Bold,
                color = Color(0xFF2D3748)
            )
            
            Spacer(modifier = Modifier.height(24.dp))
            
            // 方向控制盘
            Box(
                modifier = Modifier.size(220.dp),
                contentAlignment = Alignment.Center
            ) {
                // 外圆环
                Box(
                    modifier = Modifier
                        .size(220.dp)
                        .background(
                            Brush.radialGradient(
                                colors = listOf(
                                    Color(0xFFF7FAFC),
                                    Color(0xFFE2E8F0)
                                )
                            ),
                            CircleShape
                        )
                        .shadow(8.dp, CircleShape)
                )
                
                // 内圆环
                Box(
                    modifier = Modifier
                        .size(180.dp)
                        .background(
                            Color.White,
                            CircleShape
                        )
                        .shadow(4.dp, CircleShape)
                )
                
                // 方向按钮
                // 前进
                Box(
                    modifier = Modifier.offset(y = (-70).dp)
                ) {
                    DirectionButton(
                        text = "↑",
                        label = "FORWARD",
                        onClick = onForward,
                        color = Color(0xFF48BB78)
                    )
                }
                
                // 后退
                Box(
                    modifier = Modifier.offset(y = 70.dp)
                ) {
                    DirectionButton(
                        text = "↓",
                        label = "BACKWARD",
                        onClick = onBackward,
                        color = Color(0xFF3182CE)
                    )
                }
                
                // 左转
                Box(
                    modifier = Modifier.offset(x = (-70).dp)
                ) {
                    DirectionButton(
                        text = "←",
                        label = "LEFT",
                        onClick = onLeft,
                        color = Color(0xFFED8936)
                    )
                }
                
                // 右转
                Box(
                    modifier = Modifier.offset(x = 70.dp)
                ) {
                    DirectionButton(
                        text = "→",
                        label = "RIGHT",
                        onClick = onRight,
                        color = Color(0xFF9F7AEA)
                    )
                }
                
                // 中心停止按钮
                Button(
                    onClick = onStop,
                    modifier = Modifier.size(50.dp),
                    colors = ButtonDefaults.buttonColors(
                        containerColor = Color(0xFFE53E3E)
                    ),
                    shape = CircleShape,
                    contentPadding = PaddingValues(0.dp)
                ) {
                    Text(
                        text = "STOP",
                        color = Color.White,
                        fontSize = 10.sp,
                        fontWeight = FontWeight.Bold,
                        textAlign = TextAlign.Center
                    )
                }
            }
        }
    }
}

@Composable
fun DirectionButton(
    text: String,
    label: String,
    onClick: () -> Unit,
    color: Color
) {
    Column(
        horizontalAlignment = Alignment.CenterHorizontally
    ) {
        Button(
            onClick = onClick,
            modifier = Modifier.size(50.dp),
            colors = ButtonDefaults.buttonColors(
                containerColor = color
            ),
            shape = CircleShape,
            contentPadding = PaddingValues(0.dp)
        ) {
            Text(
                text = text,
                color = Color.White,
                fontSize = 24.sp,
                fontWeight = FontWeight.Bold
            )
        }
        
        Spacer(modifier = Modifier.height(4.dp))
        
        Text(
            text = label,
            fontSize = 10.sp,
            color = Color(0xFF718096),
            fontWeight = FontWeight.Medium
        )
    }
}

@Composable
fun SpeedControlCard(
    currentSpeed: Float,
    onSpeedChange: (Float) -> Unit
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
            modifier = Modifier.padding(24.dp)
        ) {
            Text(
                text = "Speed Control",
                fontSize = 20.sp,
                fontWeight = FontWeight.Bold,
                color = Color(0xFF2D3748)
            )
            
            Spacer(modifier = Modifier.height(16.dp))
            
            Text(
                text = "Current Speed: ${String.format("%.2f", currentSpeed)}",
                fontSize = 16.sp,
                color = Color(0xFF4A5568),
                fontWeight = FontWeight.Medium
            )
            
            Spacer(modifier = Modifier.height(16.dp))
            
            Slider(
                value = currentSpeed,
                onValueChange = onSpeedChange,
                valueRange = 0f..1f,
                steps = 19,
                colors = SliderDefaults.colors(
                    thumbColor = Color(0xFF3182CE),
                    activeTrackColor = Color(0xFF3182CE),
                    inactiveTrackColor = Color(0xFFE2E8F0)
                )
            )
            
            Row(
                modifier = Modifier.fillMaxWidth(),
                horizontalArrangement = Arrangement.SpaceBetween
            ) {
                Text(
                    text = "0.0",
                    fontSize = 12.sp,
                    color = Color(0xFF718096)
                )
                Text(
                    text = "1.0",
                    fontSize = 12.sp,
                    color = Color(0xFF718096)
                )
            }
        }
    }
}

@Composable
fun AngleControlCard(
    currentAngle: Float,
    onAngleChange: (Float) -> Unit
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
            modifier = Modifier.padding(24.dp)
        ) {
            Text(
                text = "Angle Control",
                fontSize = 20.sp,
                fontWeight = FontWeight.Bold,
                color = Color(0xFF2D3748)
            )
            
            Spacer(modifier = Modifier.height(16.dp))
            
            Text(
                text = "Target Angle: ${String.format("%.1f", currentAngle)}°",
                fontSize = 16.sp,
                color = Color(0xFF4A5568),
                fontWeight = FontWeight.Medium
            )
            
            Spacer(modifier = Modifier.height(16.dp))
            
            Slider(
                value = currentAngle,
                onValueChange = onAngleChange,
                valueRange = -30f..30f,
                steps = 59,
                colors = SliderDefaults.colors(
                    thumbColor = Color(0xFF9F7AEA),
                    activeTrackColor = Color(0xFF9F7AEA),
                    inactiveTrackColor = Color(0xFFE2E8F0)
                )
            )
            
            Row(
                modifier = Modifier.fillMaxWidth(),
                horizontalArrangement = Arrangement.SpaceBetween
            ) {
                Text(
                    text = "-30°",
                    fontSize = 12.sp,
                    color = Color(0xFF718096)
                )
                Text(
                    text = "0°",
                    fontSize = 12.sp,
                    color = Color(0xFF718096)
                )
                Text(
                    text = "30°",
                    fontSize = 12.sp,
                    color = Color(0xFF718096)
                )
            }
        }
    }
}

@Composable
fun DataDisplayCard(
    data: String,
    title: String
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
            modifier = Modifier.padding(24.dp)
        ) {
            Text(
                text = title,
                fontSize = 20.sp,
                fontWeight = FontWeight.Bold,
                color = Color(0xFF2D3748)
            )
            
            Spacer(modifier = Modifier.height(16.dp))
            
            Text(
                text = data,
                fontSize = 14.sp,
                color = Color(0xFF4A5568),
                modifier = Modifier
                    .fillMaxWidth()
                    .background(
                        Color(0xFFF7FAFC),
                        RoundedCornerShape(12.dp)
                    )
                    .padding(16.dp)
            )
        }
    }
}

@Composable
fun PIDSettingsDialog(
    onDismiss: () -> Unit,
    onConfirm: (Float, Float, Float) -> Unit
) {
    var kp by remember { mutableStateOf("1.0") }
    var ki by remember { mutableStateOf("0.1") }
    var kd by remember { mutableStateOf("0.05") }
    
    AlertDialog(
        onDismissRequest = onDismiss,
        title = {
            Text(
                text = "PID Parameters",
                fontWeight = FontWeight.Bold
            )
        },
        text = {
            Column {
                OutlinedTextField(
                    value = kp,
                    onValueChange = { kp = it },
                    label = { Text("Kp (Proportional)") },
                    modifier = Modifier.fillMaxWidth()
                )
                
                Spacer(modifier = Modifier.height(8.dp))
                
                OutlinedTextField(
                    value = ki,
                    onValueChange = { ki = it },
                    label = { Text("Ki (Integral)") },
                    modifier = Modifier.fillMaxWidth()
                )
                
                Spacer(modifier = Modifier.height(8.dp))
                
                OutlinedTextField(
                    value = kd,
                    onValueChange = { kd = it },
                    label = { Text("Kd (Derivative)") },
                    modifier = Modifier.fillMaxWidth()
                )
            }
        },
        confirmButton = {
            TextButton(
                onClick = {
                    try {
                        val kpFloat = kp.toFloat()
                        val kiFloat = ki.toFloat()
                        val kdFloat = kd.toFloat()
                        onConfirm(kpFloat, kiFloat, kdFloat)
                    } catch (e: NumberFormatException) {
                        // 处理输入错误
                    }
                }
            ) {
                Text("Confirm")
            }
        },
        dismissButton = {
            TextButton(onClick = onDismiss) {
                Text("Cancel")
            }
        }
    )
}
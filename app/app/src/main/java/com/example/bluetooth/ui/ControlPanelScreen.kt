package com.example.bluetooth.ui

import androidx.compose.foundation.background
import androidx.compose.foundation.layout.*
import androidx.compose.foundation.lazy.LazyColumn
import androidx.compose.foundation.lazy.itemsIndexed
import androidx.compose.foundation.lazy.items
import androidx.compose.foundation.lazy.rememberLazyListState
import androidx.compose.foundation.shape.CircleShape
import androidx.compose.foundation.shape.RoundedCornerShape
import androidx.compose.material.icons.Icons
import androidx.compose.material.icons.filled.ArrowBack
import androidx.compose.material.icons.filled.Settings
import androidx.compose.material.icons.filled.Bluetooth
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
    onBackClick: () -> Unit
) {
    val receivedData by bluetoothManager.receivedData.collectAsState()
    val connectionState by bluetoothManager.connectionState.collectAsState()
    val isConnected = connectionState is CustomBluetoothManager.ConnectionState.CONNECTED
    
    var currentSpeed by remember { mutableFloatStateOf(200f) }
    var currentAngle by remember { mutableFloatStateOf(0f) }
    var isBalanceEnabled by remember { mutableStateOf(false) }
    
    val scope = rememberCoroutineScope()
    
    // 控制函数
    val moveForward = {
        scope.launch {
            bluetoothManager.moveForward(currentSpeed / 1000f)
        }.let {}
    }
    
    val moveBackward = {
        scope.launch {
            bluetoothManager.moveBackward(currentSpeed / 1000f)
        }.let {}
    }
    
    val turnLeft = {
        scope.launch {
            bluetoothManager.turnLeft()
        }.let {}
    }
    
    val turnRight = {
        scope.launch {
            bluetoothManager.turnRight()
        }.let {}
    }
    
    val stopMovement = {
        scope.launch {
            bluetoothManager.stopBalance()
        }.let {}
    }
    
    val setAngle = { angle: Float ->
        scope.launch {
            bluetoothManager.setAngle(angle)
        }
    }
    
    val setSpeed = { speed: Float ->
        scope.launch {
            bluetoothManager.setSpeed(speed / 1000f)
        }
    }
    
    val toggleBalance = {
        scope.launch {
            if (isBalanceEnabled) {
                bluetoothManager.stopBalance()
            } else {
                bluetoothManager.startBalance()
            }
            isBalanceEnabled = !isBalanceEnabled
        }.let {}
    }
    
    val resetSystem = {
        scope.launch {
            bluetoothManager.resetSystem()
            bluetoothManager.setSpeed(currentSpeed / 1000f)
        }.let {}
    }
    
    val getStatus = {
        scope.launch {
            bluetoothManager.getStatus()
        }.let {}
    }
    
    Scaffold(
        topBar = {
            TopAppBar(
                title = { 
                    Text(
                        "Balance Robot Control",
                        fontWeight = FontWeight.Bold
                    )
                },
                navigationIcon = {
                    IconButton(onClick = onBackClick) {
                        Icon(Icons.Filled.ArrowBack, contentDescription = "Back")
                    }
                },
                actions = {
                    IconButton(onClick = { /* Settings */ }) {
                        Icon(Icons.Filled.Settings, contentDescription = "Settings")
                    }
                },
                colors = TopAppBarDefaults.topAppBarColors(
                    containerColor = MaterialTheme.colorScheme.primaryContainer
                )
            )
        }
    ) { paddingValues ->
        LazyColumn(
            modifier = Modifier
                .fillMaxSize()
                .padding(paddingValues)
                .padding(16.dp),
            verticalArrangement = Arrangement.spacedBy(16.dp)
        ) {
            // 连接状态指示器
            item {
                ConnectionStatusCard(isConnected = isConnected)
            }
            
            // HC-05蓝牙消息显示卡片
            item {
                BluetoothMessageCard(
                    receivedData = receivedData
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
            
            // 角度控制卡片
            item {
                AngleControlCard(
                    currentAngle = currentAngle,
                    onAngleChange = { angle ->
                        currentAngle = angle
                        setAngle(angle)
                    }
                )
            }
            
            // 系统状态控制卡片
            item {
                SystemStatusCard(
                    isBalanceEnabled = isBalanceEnabled,
                    onToggleBalance = toggleBalance,
                    onReset = resetSystem,
                    onGetStatus = getStatus
                )
            }
            
            // 方向控制卡片
            item {
                DirectionControlCard(
                    onMoveForward = moveForward,
                    onMoveBackward = moveBackward,
                    onTurnLeft = turnLeft,
                    onTurnRight = turnRight,
                    onStop = stopMovement
                )
            }
            
            // 速度控制卡片
            item {
                SpeedControlCard(
                    currentSpeed = currentSpeed,
                    onSpeedChange = { speed ->
                        currentSpeed = speed
                        setSpeed(speed)
                    }
                )
            }
        }
    }
}

@Composable
fun ConnectionStatusCard(isConnected: Boolean) {
    Card(
        modifier = Modifier.fillMaxWidth(),
        colors = CardDefaults.cardColors(
            containerColor = if (isConnected) {
                Color(0xFF4CAF50).copy(alpha = 0.1f)
            } else {
                Color(0xFFF44336).copy(alpha = 0.1f)
            }
        )
    ) {
        Row(
            modifier = Modifier
                .fillMaxWidth()
                .padding(16.dp),
            verticalAlignment = Alignment.CenterVertically
        ) {
            Box(
                modifier = Modifier
                    .size(12.dp)
                    .background(
                        color = if (isConnected) Color(0xFF4CAF50) else Color(0xFFF44336),
                        shape = CircleShape
                    )
            )
            Spacer(modifier = Modifier.width(12.dp))
            Text(
                text = if (isConnected) "Connected to Robot" else "Disconnected",
                style = MaterialTheme.typography.bodyLarge,
                fontWeight = FontWeight.Medium
            )
        }
    }
}

@Composable
fun BluetoothMessageCard(
    receivedData: String,
    modifier: Modifier = Modifier
) {
    val listState = rememberLazyListState()
    val messages = remember(receivedData) {
        if (receivedData.isNotEmpty()) {
            receivedData.split("\n").filter { it.isNotBlank() }
        } else {
            emptyList()
        }
    }
    
    // 自动滚动到最新消息
    LaunchedEffect(messages.size) {
        if (messages.isNotEmpty()) {
            listState.animateScrollToItem(messages.size - 1)
        }
    }
    
    Card(
        modifier = modifier
            .fillMaxWidth()
            .height(200.dp),
        colors = CardDefaults.cardColors(
            containerColor = MaterialTheme.colorScheme.surfaceVariant
        ),
        elevation = CardDefaults.cardElevation(defaultElevation = 4.dp)
    ) {
        Column(
            modifier = Modifier
                .fillMaxSize()
                .padding(16.dp)
        ) {
            Row(
                verticalAlignment = Alignment.CenterVertically,
                modifier = Modifier.fillMaxWidth()
            ) {
                Icon(
                    imageVector = Icons.Filled.Bluetooth,
                    contentDescription = "Bluetooth Messages",
                    tint = MaterialTheme.colorScheme.primary,
                    modifier = Modifier.size(24.dp)
                )
                Spacer(modifier = Modifier.width(8.dp))
                Text(
                    text = "HC-05 蓝牙消息",
                    style = MaterialTheme.typography.titleMedium,
                    fontWeight = FontWeight.Bold
                )
                Spacer(modifier = Modifier.weight(1f))
                Text(
                    text = "${messages.size} 条消息",
                    style = MaterialTheme.typography.bodySmall,
                    color = MaterialTheme.colorScheme.onSurfaceVariant
                )
            }
            
            Spacer(modifier = Modifier.height(12.dp))
            
            Card(
                modifier = Modifier.fillMaxSize(),
                colors = CardDefaults.cardColors(
                    containerColor = MaterialTheme.colorScheme.surface
                ),
                elevation = CardDefaults.cardElevation(defaultElevation = 2.dp)
            ) {
                if (messages.isEmpty()) {
                    Box(
                        modifier = Modifier.fillMaxSize(),
                        contentAlignment = Alignment.Center
                    ) {
                        Column(
                            horizontalAlignment = Alignment.CenterHorizontally
                        ) {
                            Icon(
                                imageVector = Icons.Filled.Bluetooth,
                                contentDescription = null,
                                tint = MaterialTheme.colorScheme.onSurfaceVariant.copy(alpha = 0.5f),
                                modifier = Modifier.size(32.dp)
                            )
                            Spacer(modifier = Modifier.height(8.dp))
                            Text(
                                text = "等待蓝牙消息...",
                                style = MaterialTheme.typography.bodyMedium,
                                color = MaterialTheme.colorScheme.onSurfaceVariant.copy(alpha = 0.7f)
                            )
                        }
                    }
                } else {
                    LazyColumn(
                        state = listState,
                        modifier = Modifier
                            .fillMaxSize()
                            .padding(8.dp),
                        verticalArrangement = Arrangement.spacedBy(4.dp)
                    ) {
                        itemsIndexed(messages) { index, message ->
                            val isLatest = index == messages.size - 1
                            
                            Card(
                                modifier = Modifier.fillMaxWidth(),
                                colors = CardDefaults.cardColors(
                                    containerColor = if (isLatest) {
                                        MaterialTheme.colorScheme.primaryContainer
                                    } else {
                                        MaterialTheme.colorScheme.surfaceVariant
                                    }
                                ),
                                elevation = CardDefaults.cardElevation(
                                    defaultElevation = if (isLatest) 2.dp else 1.dp
                                )
                            ) {
                                Row(
                                    modifier = Modifier
                                        .fillMaxWidth()
                                        .padding(8.dp),
                                    verticalAlignment = Alignment.CenterVertically
                                ) {
                                    Text(
                                        text = (index + 1).toString() + ".",
                                        style = MaterialTheme.typography.bodySmall,
                                        color = MaterialTheme.colorScheme.onSurfaceVariant,
                                        modifier = Modifier.width(24.dp)
                                    )
                                    Text(
                                        text = message,
                                        style = MaterialTheme.typography.bodyMedium,
                                        color = if (isLatest) {
                                            MaterialTheme.colorScheme.onPrimaryContainer
                                        } else {
                                            MaterialTheme.colorScheme.onSurfaceVariant
                                        },
                                        modifier = Modifier.weight(1f)
                                    )
                                } // Row
                            } // Card
                        } // itemsIndexed
                    } // LazyColumn
                }
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
        modifier = Modifier.fillMaxWidth(),
        colors = CardDefaults.cardColors(
            containerColor = MaterialTheme.colorScheme.surfaceVariant
        )
    ) {
        Column(
            modifier = Modifier
                .fillMaxWidth()
                .padding(16.dp)
        ) {
            Text(
                text = title,
                style = MaterialTheme.typography.titleMedium,
                fontWeight = FontWeight.Bold
            )
            Spacer(modifier = Modifier.height(8.dp))
            Text(
                text = data,
                style = MaterialTheme.typography.bodyMedium,
                modifier = Modifier
                    .fillMaxWidth()
                    .background(
                        MaterialTheme.colorScheme.surface,
                        RoundedCornerShape(8.dp)
                    )
                    .padding(12.dp)
            )
        }
    }
}

@Composable
fun AngleControlCard(
    currentAngle: Float,
    onAngleChange: (Float) -> Unit
) {
    Card(
        modifier = Modifier.fillMaxWidth(),
        colors = CardDefaults.cardColors(
            containerColor = MaterialTheme.colorScheme.surfaceVariant
        )
    ) {
        Column(
            modifier = Modifier
                .fillMaxWidth()
                .padding(16.dp)
        ) {
            Text(
                text = "Angle Control",
                style = MaterialTheme.typography.titleMedium,
                fontWeight = FontWeight.Bold
            )
            Spacer(modifier = Modifier.height(16.dp))
            
            Text(
                text = "Current Angle: ${currentAngle.toInt()}°",
                style = MaterialTheme.typography.bodyLarge,
                textAlign = TextAlign.Center,
                modifier = Modifier.fillMaxWidth()
            )
            
            Spacer(modifier = Modifier.height(16.dp))
            
            Slider(
                value = currentAngle,
                onValueChange = onAngleChange,
                valueRange = -45f..45f,
                steps = 89,
                modifier = Modifier.fillMaxWidth()
            )
            
            Row(
                modifier = Modifier.fillMaxWidth(),
                horizontalArrangement = Arrangement.SpaceBetween
            ) {
                Text(
                    text = "-45°",
                    style = MaterialTheme.typography.bodySmall,
                    color = MaterialTheme.colorScheme.onSurfaceVariant
                )
                Text(
                    text = "45°",
                    style = MaterialTheme.typography.bodySmall,
                    color = MaterialTheme.colorScheme.onSurfaceVariant
                )
            }
        }
    }
}

@Composable
fun SystemStatusCard(
    isBalanceEnabled: Boolean,
    onToggleBalance: () -> Unit,
    onReset: () -> Unit,
    onGetStatus: () -> Unit
) {
    Card(
        modifier = Modifier.fillMaxWidth(),
        colors = CardDefaults.cardColors(
            containerColor = MaterialTheme.colorScheme.surfaceVariant
        )
    ) {
        Column(
            modifier = Modifier
                .fillMaxWidth()
                .padding(16.dp)
        ) {
            Text(
                text = "System Control",
                style = MaterialTheme.typography.titleMedium,
                fontWeight = FontWeight.Bold
            )
            Spacer(modifier = Modifier.height(16.dp))
            
            Row(
                modifier = Modifier.fillMaxWidth(),
                horizontalArrangement = Arrangement.spacedBy(8.dp)
            ) {
                Button(
                    onClick = onToggleBalance,
                    modifier = Modifier.weight(1f),
                    colors = ButtonDefaults.buttonColors(
                        containerColor = if (isBalanceEnabled) {
                            MaterialTheme.colorScheme.error
                        } else {
                            MaterialTheme.colorScheme.primary
                        }
                    )
                ) {
                    Text(
                        text = if (isBalanceEnabled) "Disable Balance" else "Enable Balance",
                        fontSize = 12.sp
                    )
                }
                
                OutlinedButton(
                    onClick = onReset,
                    modifier = Modifier.weight(1f)
                ) {
                    Text("Reset", fontSize = 12.sp)
                }
                
                OutlinedButton(
                    onClick = onGetStatus,
                    modifier = Modifier.weight(1f)
                ) {
                    Text("Status", fontSize = 12.sp)
                }
            }
        }
    }
}

@Composable
fun DirectionControlCard(
    onMoveForward: () -> Unit,
    onMoveBackward: () -> Unit,
    onTurnLeft: () -> Unit,
    onTurnRight: () -> Unit,
    onStop: () -> Unit
) {
    Card(
        modifier = Modifier.fillMaxWidth(),
        colors = CardDefaults.cardColors(
            containerColor = MaterialTheme.colorScheme.surfaceVariant
        )
    ) {
        Column(
            modifier = Modifier
                .fillMaxWidth()
                .padding(16.dp),
            horizontalAlignment = Alignment.CenterHorizontally
        ) {
            Text(
                text = "Direction Control",
                style = MaterialTheme.typography.titleMedium,
                fontWeight = FontWeight.Bold
            )
            Spacer(modifier = Modifier.height(16.dp))
            
            // 十字方向控制布局
            Column(
                horizontalAlignment = Alignment.CenterHorizontally,
                verticalArrangement = Arrangement.spacedBy(8.dp)
            ) {
                // 前进按钮
//                DirectionButton(
//                    text = "↑",
//                    label = "Forward",
//                    onClick = onMoveForward
//                )
                
                // 左转、停止、右转按钮
                Row(
                    horizontalArrangement = Arrangement.spacedBy(8.dp),
                    verticalAlignment = Alignment.CenterVertically
                ) {
//                    DirectionButton(
//                        text = "←",
//                        label = "Left",
//                        onClick = onTurnLeft
//                    )
                    
                    Button(
                        onClick = onStop,
                        modifier = Modifier.size(60.dp),
                        shape = CircleShape,
                        colors = ButtonDefaults.buttonColors(
                            containerColor = MaterialTheme.colorScheme.error
                        ),
                        contentPadding = PaddingValues(0.dp)
                    ) {
                        Text(
                            text = "STOP",
                            fontSize = 10.sp,
                            fontWeight = FontWeight.Bold
                        )
                    }
                    
//                    DirectionButton(
//                        text = "→",
//                        label = "Right",
//                        onClick = onTurnRight
//                    )
                }
                
                // 后退按钮
//                DirectionButton(
//                    text = "↓",
//                    label = "Backward",
//                    onClick = onMoveBackward
//                )
            }
        }
    }
}

@Composable
fun DirectionButton(
    text: String,
    label: String,
    onClick: () -> Unit
) {
    Column(
        horizontalAlignment = Alignment.CenterHorizontally
    ) {
        Button(
            onClick = onClick,
            modifier = Modifier.size(60.dp),
            shape = CircleShape,
            colors = ButtonDefaults.buttonColors(
                containerColor = MaterialTheme.colorScheme.primary
            ),
            contentPadding = PaddingValues(0.dp)
        ) {
            Text(
                text = text,
                fontSize = 20.sp,
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
        modifier = Modifier.fillMaxWidth(),
        colors = CardDefaults.cardColors(
            containerColor = MaterialTheme.colorScheme.surfaceVariant
        )
    ) {
        Column(
            modifier = Modifier
                .fillMaxWidth()
                .padding(16.dp)
        ) {
            Text(
                text = "Speed Control",
                style = MaterialTheme.typography.titleMedium,
                fontWeight = FontWeight.Bold
            )
            Spacer(modifier = Modifier.height(16.dp))
            
            Text(
                text = "Current Speed: ${currentSpeed.toInt()}",
                style = MaterialTheme.typography.bodyLarge,
                textAlign = TextAlign.Center,
                modifier = Modifier.fillMaxWidth()
            )
            
            Spacer(modifier = Modifier.height(16.dp))
            
            Slider(
                value = currentSpeed,
                onValueChange = onSpeedChange,
                valueRange = 0f..1000f,
                steps = 99,
                modifier = Modifier.fillMaxWidth()
            )
            
            Row(
                modifier = Modifier.fillMaxWidth(),
                horizontalArrangement = Arrangement.SpaceBetween
            ) {
                Text(
                    text = "0",
                    style = MaterialTheme.typography.bodySmall,
                    color = MaterialTheme.colorScheme.onSurfaceVariant
                )
                Text(
                    text = "1000",
                    style = MaterialTheme.typography.bodySmall,
                    color = MaterialTheme.colorScheme.onSurfaceVariant
                )
            }
        }
    }
}
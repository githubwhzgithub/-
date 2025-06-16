package com.example.bluetooth.ui

import androidx.compose.foundation.background
import androidx.compose.foundation.layout.*
import androidx.compose.foundation.lazy.LazyColumn
import androidx.compose.foundation.lazy.itemsIndexed
import androidx.compose.foundation.lazy.items
import androidx.compose.foundation.lazy.rememberLazyListState
import androidx.compose.foundation.lazy.grid.LazyVerticalGrid
import androidx.compose.foundation.lazy.grid.GridCells
import androidx.compose.foundation.shape.CircleShape
import androidx.compose.foundation.shape.RoundedCornerShape
import androidx.compose.material.icons.Icons
import androidx.compose.material.icons.filled.ArrowBack
import androidx.compose.material.icons.filled.Settings
import androidx.compose.material.icons.filled.Bluetooth
import androidx.compose.material.icons.filled.Dashboard
import androidx.compose.material.icons.filled.RotateLeft
import androidx.compose.material.icons.filled.RotateRight
import androidx.compose.material.icons.filled.Speed
import androidx.compose.material.icons.filled.Straighten
import androidx.compose.material.icons.filled.CheckCircle
import androidx.compose.material.icons.filled.Cancel
import androidx.compose.material.icons.filled.SwapHoriz
import androidx.compose.material.icons.filled.SwapVert
import androidx.compose.material.icons.filled.Timeline
import androidx.compose.material.icons.filled.HorizontalRule
import androidx.compose.material.icons.filled.CropFree
import androidx.compose.material.icons.filled.CropSquare
import androidx.compose.material.icons.filled.Visibility
import androidx.compose.material.icons.filled.VisibilityOff
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

@OptIn(ExperimentalMaterial3Api::class)
@Composable
fun ControlPanelScreen(
    bluetoothManager: CustomBluetoothManager,
    onBackClick: () -> Unit
) {
    val receivedData by bluetoothManager.receivedData.collectAsState()
    val robotStatus by bluetoothManager.robotStatus.collectAsState()
    val connectionState by bluetoothManager.connectionState.collectAsState()
    val isConnected = connectionState is CustomBluetoothManager.ConnectionState.CONNECTED
    
    var currentSpeed by remember { mutableFloatStateOf(200f) }
    var currentAngle by remember { mutableFloatStateOf(0f) }
    var isBalanceEnabled by remember { mutableStateOf(false) }
    var visionMode by remember { mutableIntStateOf(0) } // 0:关闭, 1:循迹, 2:物体追踪
    
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
    
    val setVisionMode = { mode: Int ->
        scope.launch {
            bluetoothManager.setVisionMode(mode)
            visionMode = mode
        }.let {}
    }
    
    val enableLineTracking = {
        scope.launch {
            bluetoothManager.enableLineTracking()
            visionMode = 1
        }.let {}
    }
    
    val enableObjectTracking = {
        scope.launch {
            bluetoothManager.enableObjectTracking()
            visionMode = 2
        }.let {}
    }
    
    val disableVision = {
        scope.launch {
            bluetoothManager.disableVision()
            visionMode = 0
        }.let {}
    }
    
    Scaffold(
        topBar = {
            TopAppBar(
                title = { 
                    Text(
                        "Balance Car Control",
                        fontWeight = FontWeight.Bold
                    )
                },
                navigationIcon = {
                    IconButton(onClick = onBackClick) {
                        Icon(Icons.Filled.ArrowBack, contentDescription = "Back")
                    }
                },
                actions = {
                    // 右上角设置部分已移除
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
            
            // 机器人状态显示卡片 - 根据蓝牙数据解析显示
            item {
                RobotStatusCard(robotStatus = robotStatus)
            }
            
            // GO按钮 - 根据蓝牙协议添加
            item {
                GoButtonCard(
                    onSendGo = {
                        scope.launch {
                            bluetoothManager.sendCommand("GO")
                        }
                    }
                )
            }
            
            // START按钮 - 开始平衡控制
            item {
                StartButtonCard(
                    isBalanceEnabled = isBalanceEnabled,
                    onToggleBalance = toggleBalance
                )
            }
            
            // 系统状态控制卡片 (只保留Reset按钮)
            item {
                SystemResetCard(onReset = resetSystem)
            }
            
            // 方向控制卡片
            item {
                DirectionControlCard(
                    onMoveForward = moveForward,
                    onMoveBackward = moveBackward,
                    onTurnLeft = turnLeft,
                    onTurnRight = turnRight,
                    onStop = stopMovement,
                    onStopRotation = {
                        scope.launch {
                            bluetoothManager.sendCommand("LEFT 0")
                        }
                    }
                )
            }
            
            // 视觉模式设置卡片 (速度控制条已移除)
            item {
                VisionModeCard(
                    currentMode = visionMode,
                    onModeChange = setVisionMode,
                    onLineTracking = enableLineTracking,
                    onObjectTracking = enableObjectTracking,
                    onDisableVision = disableVision
                )
            }
        }
    }
}

@Composable
fun GoButtonCard(
    onSendGo: () -> Unit
) {
    Card(
        modifier = Modifier.fillMaxWidth(),
        colors = CardDefaults.cardColors(
            containerColor = MaterialTheme.colorScheme.primaryContainer
        ),
        elevation = CardDefaults.cardElevation(defaultElevation = 4.dp)
    ) {
        Column(
            modifier = Modifier
                .fillMaxWidth()
                .padding(16.dp),
            horizontalAlignment = Alignment.CenterHorizontally
        ) {
            Text(
                text = "连接控制",
                style = MaterialTheme.typography.titleMedium,
                fontWeight = FontWeight.Bold
            )
            
            Spacer(modifier = Modifier.height(16.dp))
            
            Button(
                onClick = onSendGo,
                modifier = Modifier
                    .fillMaxWidth()
                    .height(56.dp),
                colors = ButtonDefaults.buttonColors(
                    containerColor = Color(0xFF4CAF50)
                ),
                shape = RoundedCornerShape(12.dp)
            ) {
                Row(
                    verticalAlignment = Alignment.CenterVertically,
                    horizontalArrangement = Arrangement.Center
                ) {
                    Icon(
                        imageVector = Icons.Filled.CheckCircle,
                        contentDescription = "Send GO",
                        tint = Color.White,
                        modifier = Modifier.size(24.dp)
                    )
                    Spacer(modifier = Modifier.width(8.dp))
                    Text(
                        text = "发送 GO 指令",
                        fontSize = 18.sp,
                        fontWeight = FontWeight.Bold,
                        color = Color.White
                    )
                }
            }
            
            Spacer(modifier = Modifier.height(8.dp))
            
            Text(
                text = "发送GO指令以建立与平衡车的连接",
                style = MaterialTheme.typography.bodySmall,
                color = MaterialTheme.colorScheme.onSurfaceVariant,
                textAlign = TextAlign.Center
            )
        }
    }
}

@Composable
fun StartButtonCard(
    isBalanceEnabled: Boolean,
    onToggleBalance: () -> Unit
) {
    Card(
        modifier = Modifier.fillMaxWidth(),
        colors = CardDefaults.cardColors(
            containerColor = MaterialTheme.colorScheme.secondaryContainer
        ),
        elevation = CardDefaults.cardElevation(defaultElevation = 4.dp)
    ) {
        Column(
            modifier = Modifier
                .fillMaxWidth()
                .padding(16.dp),
            horizontalAlignment = Alignment.CenterHorizontally
        ) {
            Text(
                text = "平衡控制",
                style = MaterialTheme.typography.titleMedium,
                fontWeight = FontWeight.Bold
            )
            
            Spacer(modifier = Modifier.height(16.dp))
            
            Button(
                onClick = onToggleBalance,
                modifier = Modifier
                    .fillMaxWidth()
                    .height(56.dp),
                colors = ButtonDefaults.buttonColors(
                    containerColor = if (isBalanceEnabled) {
                        Color(0xFFF44336) // 红色 - 停止
                    } else {
                        Color(0xFF2196F3) // 蓝色 - 开始
                    }
                ),
                shape = RoundedCornerShape(12.dp)
            ) {
                Row(
                    verticalAlignment = Alignment.CenterVertically,
                    horizontalArrangement = Arrangement.Center
                ) {
                    Icon(
                        imageVector = if (isBalanceEnabled) {
                            Icons.Filled.Cancel
                        } else {
                            Icons.Filled.CheckCircle
                        },
                        contentDescription = if (isBalanceEnabled) "Stop Balance" else "Start Balance",
                        tint = Color.White,
                        modifier = Modifier.size(24.dp)
                    )
                    Spacer(modifier = Modifier.width(8.dp))
                    Text(
                        text = if (isBalanceEnabled) {
                            "停止平衡"
                        } else {
                            "开始平衡"
                        },
                        fontSize = 18.sp,
                        fontWeight = FontWeight.Bold,
                        color = Color.White
                    )
                }
            }
            
            Spacer(modifier = Modifier.height(8.dp))
            
            Text(
                text = if (isBalanceEnabled) {
                    "点击停止平衡车的平衡控制"
                } else {
                    "点击开始平衡车的平衡控制"
                },
                style = MaterialTheme.typography.bodySmall,
                color = MaterialTheme.colorScheme.onSurfaceVariant,
                textAlign = TextAlign.Center
            )
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
fun SystemResetCard(
    onReset: () -> Unit
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
            
            OutlinedButton(
                onClick = onReset,
                modifier = Modifier.fillMaxWidth()
            ) {
                Text("Reset System", fontSize = 14.sp)
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
    onStop: () -> Unit,
    onStopRotation: () -> Unit // 新增停止转动回调
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
                DirectionButton(
                    text = "↑",
                    label = "Forward",
                    onClick = onMoveForward
                )
                
                // 左转、停止、右转按钮
                Row(
                    horizontalArrangement = Arrangement.spacedBy(8.dp),
                    verticalAlignment = Alignment.CenterVertically
                ) {
                    DirectionButton(
                        text = "←",
                        label = "Left",
                        onClick = onTurnLeft
                    )
                    
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
                    
                    DirectionButton(
                        text = "→",
                        label = "Right",
                        onClick = onTurnRight
                    )
                }
                
                // 后退按钮
                DirectionButton(
                    text = "↓",
                    label = "Backward",
                    onClick = onMoveBackward
                )
            }
            Spacer(modifier = Modifier.height(16.dp))
            // 新增停止转动按钮
            Button(
                onClick = onStopRotation,
                modifier = Modifier.fillMaxWidth(),
                colors = ButtonDefaults.buttonColors(
                    containerColor = MaterialTheme.colorScheme.secondary
                )
            ) {
                Text("停止转动 ")
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
fun RobotStatusCard(robotStatus: com.example.bluetooth.RobotStatus) {
    Card(
        modifier = Modifier.fillMaxWidth(),
        colors = CardDefaults.cardColors(
            containerColor = MaterialTheme.colorScheme.surfaceVariant
        )
    ) {
        Column(
            modifier = Modifier.padding(16.dp)
        ) {
            Row(
                verticalAlignment = Alignment.CenterVertically,
                modifier = Modifier.fillMaxWidth()
            ) {
                Icon(
                    imageVector = Icons.Filled.Dashboard,
                    contentDescription = "Car Status",
                    tint = MaterialTheme.colorScheme.primary,
                    modifier = Modifier.size(24.dp)
                )
                Spacer(modifier = Modifier.width(8.dp))
                Text(
                    text = "平衡车状态",
                    style = MaterialTheme.typography.titleMedium,
                    fontWeight = FontWeight.Bold
                )
                Spacer(modifier = Modifier.weight(1f))
                // 状态更新时间
                val timeDiff = (System.currentTimeMillis() - robotStatus.lastUpdateTime) / 1000
                Text(
                    text = if (timeDiff < 5) "实时" else "${timeDiff}秒前",
                    style = MaterialTheme.typography.bodySmall,
                    color = if (timeDiff < 5) Color(0xFF4CAF50) else MaterialTheme.colorScheme.onSurfaceVariant
                )
            }
            
            Spacer(modifier = Modifier.height(16.dp))
            
            // 基本状态信息
            LazyVerticalGrid(
                columns = GridCells.Fixed(2),
                horizontalArrangement = Arrangement.spacedBy(8.dp),
                verticalArrangement = Arrangement.spacedBy(8.dp),
                modifier = Modifier.height(280.dp)
            ) {
                item {
                    StatusItem(
                        label = "俯仰角",
                        value = "${String.format("%.2f", robotStatus.pitch)}°",
                        icon = Icons.Filled.RotateLeft,
                        color = if (kotlin.math.abs(robotStatus.pitch) > 10) Color(0xFFF44336) else Color(0xFF4CAF50)
                    )
                }
                item {
                    StatusItem(
                        label = "横滚角",
                        value = "${String.format("%.2f", robotStatus.roll)}°",
                        icon = Icons.Filled.RotateRight,
                        color = if (kotlin.math.abs(robotStatus.roll) > 10) Color(0xFFF44336) else Color(0xFF4CAF50)
                    )
                }
                item {
                    StatusItem(
                        label = "速度",
                        value = "${String.format("%.1f", robotStatus.speed)}",
                        icon = Icons.Filled.Speed,
                        color = Color(0xFF2196F3)
                    )
                }
                item {
                    StatusItem(
                        label = "距离",
                        value = "${String.format("%.1f", robotStatus.distance)}cm",
                        icon = Icons.Filled.Straighten,
                        color = if (robotStatus.distance < 10) Color(0xFFF44336) else Color(0xFF4CAF50)
                    )
                }
                item {
                    StatusItem(
                        label = "偏航率",
                        value = "${String.format("%.4f", robotStatus.yawRate)}",
                        icon = Icons.Filled.RotateLeft,
                        color = Color(0xFF9C27B0)
                    )
                }
                item {
                    StatusItem(
                        label = "目标偏航率",
                        value = "${String.format("%.4f", robotStatus.targetYawRate)}",
                        icon = Icons.Filled.RotateRight,
                        color = Color(0xFF9C27B0)
                    )
                }
            }
        }
    }
}

@Composable
fun StatusItem(
    label: String,
    value: String,
    icon: androidx.compose.ui.graphics.vector.ImageVector,
    color: Color
) {
    Card(
        modifier = Modifier.fillMaxWidth(),
        colors = CardDefaults.cardColors(
            containerColor = color.copy(alpha = 0.1f)
        ),
        elevation = CardDefaults.cardElevation(defaultElevation = 2.dp)
    ) {
        Column(
            modifier = Modifier
                .fillMaxWidth()
                .padding(12.dp),
            horizontalAlignment = Alignment.CenterHorizontally
        ) {
            Icon(
                imageVector = icon,
                contentDescription = label,
                tint = color,
                modifier = Modifier.size(20.dp)
            )
            Spacer(modifier = Modifier.height(4.dp))
            Text(
                text = label,
                style = MaterialTheme.typography.bodySmall,
                color = MaterialTheme.colorScheme.onSurfaceVariant,
                textAlign = TextAlign.Center
            )
            Spacer(modifier = Modifier.height(2.dp))
            Text(
                text = value,
                style = MaterialTheme.typography.bodyMedium,
                fontWeight = FontWeight.Bold,
                color = color,
                textAlign = TextAlign.Center
            )
        }
    }
}

@Composable
fun VisionModeCard(
    currentMode: Int,
    onModeChange: (Int) -> Unit,
    onLineTracking: () -> Unit,
    onObjectTracking: () -> Unit,
    onDisableVision: () -> Unit
) {
    Card(
        modifier = Modifier.fillMaxWidth(),
        colors = CardDefaults.cardColors(
            containerColor = MaterialTheme.colorScheme.surfaceVariant
        )
    ) {
        Column(
            modifier = Modifier.padding(16.dp)
        ) {
            Text(
                text = "视觉模式设置",
                style = MaterialTheme.typography.titleMedium,
                fontWeight = FontWeight.Bold
            )
            
            Spacer(modifier = Modifier.height(12.dp))
            
            // 当前模式显示
            Row(
                modifier = Modifier.fillMaxWidth(),
                horizontalArrangement = Arrangement.SpaceBetween,
                verticalAlignment = Alignment.CenterVertically
            ) {
                Text(
                    text = "当前模式:",
                    style = MaterialTheme.typography.bodyMedium
                )
                Text(
                    text = when (currentMode) {
                        0 -> "关闭"
                        1 -> "循迹模式"
                        2 -> "物体追踪"
                        else -> "未知"
                    },
                    style = MaterialTheme.typography.bodyMedium,
                    fontWeight = FontWeight.Bold,
                    color = when (currentMode) {
                        0 -> MaterialTheme.colorScheme.error
                        1 -> Color(0xFF2196F3)
                        2 -> Color(0xFF4CAF50)
                        else -> MaterialTheme.colorScheme.onSurface
                    }
                )
            }
            
            Spacer(modifier = Modifier.height(16.dp))
            
            // 控制按钮
            Row(
                modifier = Modifier.fillMaxWidth(),
                horizontalArrangement = Arrangement.spacedBy(8.dp)
            ) {
                Button(
                    onClick = onLineTracking,
                    modifier = Modifier.weight(1f),
                    colors = ButtonDefaults.buttonColors(
                        containerColor = if (currentMode == 1) Color(0xFF2196F3) else MaterialTheme.colorScheme.primary
                    )
                ) {
                    Text(
                        text = "循迹",
                        fontSize = 12.sp
                    )
                }
                
                Button(
                    onClick = onObjectTracking,
                    modifier = Modifier.weight(1f),
                    colors = ButtonDefaults.buttonColors(
                        containerColor = if (currentMode == 2) Color(0xFF4CAF50) else MaterialTheme.colorScheme.primary
                    )
                ) {
                    Text(
                        text = "追踪",
                        fontSize = 12.sp
                    )
                }
                
                Button(
                    onClick = onDisableVision,
                    modifier = Modifier.weight(1f),
                    colors = ButtonDefaults.buttonColors(
                        containerColor = if (currentMode == 0) MaterialTheme.colorScheme.error else MaterialTheme.colorScheme.secondary
                    )
                ) {
                    Text(
                        text = "关闭",
                        fontSize = 12.sp
                    )
                }
            }
            
            Spacer(modifier = Modifier.height(12.dp))
            
            // 模式说明
            Text(
                text = when (currentMode) {
                    0 -> "视觉功能已关闭"
                    1 -> "循迹模式：跟随地面线条行驶"
                    2 -> "物体追踪：跟随指定物体移动"
                    else -> "未知模式"
                },
                style = MaterialTheme.typography.bodySmall,
                color = MaterialTheme.colorScheme.onSurfaceVariant,
                modifier = Modifier.fillMaxWidth()
            )
        }
    }
}

// SpeedControlCard 已被移除

/*
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
*/
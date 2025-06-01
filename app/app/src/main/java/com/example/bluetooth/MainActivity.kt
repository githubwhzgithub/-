package com.example.bluetooth

import android.os.Bundle
import androidx.activity.ComponentActivity
import androidx.activity.compose.setContent
import androidx.activity.enableEdgeToEdge
import androidx.activity.result.contract.ActivityResultContracts
import androidx.compose.foundation.layout.fillMaxSize
import androidx.compose.foundation.layout.padding
import androidx.compose.material3.Scaffold
import androidx.compose.runtime.*
import androidx.compose.ui.Modifier
import androidx.lifecycle.lifecycleScope
import com.example.bluetooth.CustomBluetoothManager
import com.example.bluetooth.ui.BalanceControlScreen
import com.example.bluetooth.ui.theme.MeiFengYiHaoTheme
import kotlinx.coroutines.launch

class MainActivity : ComponentActivity() {
    private lateinit var bluetoothManager: CustomBluetoothManager
    
    // 权限请求启动器
    private val permissionLauncher = registerForActivityResult(
        ActivityResultContracts.RequestMultiplePermissions()
    ) { permissions ->
        val allGranted = permissions.values.all { it }
        if (allGranted) {
            // 权限已授予，更新设备列表
            bluetoothManager.updatePairedDevices()
        }
    }
    
    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        
        // 初始化蓝牙管理器
        bluetoothManager = CustomBluetoothManager(this)
        
        enableEdgeToEdge()
        setContent {
            MeiFengYiHaoTheme {
                Scaffold(modifier = Modifier.fillMaxSize()) { innerPadding ->
                    BalanceControlScreen(
                                bluetoothManager = bluetoothManager,
                                modifier = Modifier.padding(innerPadding)
                    )
                }
            }
        }
        
        // 检查并请求权限
        if (!bluetoothManager.hasBluetoothPermissions()) {
            requestBluetoothPermissions()
        }
    }
    
    private fun requestBluetoothPermissions() {
        val permissions = bluetoothManager.getRequiredPermissions()
        permissionLauncher.launch(permissions)
    }
    
    override fun onDestroy() {
        super.onDestroy()
        bluetoothManager.cleanup()
    }
}
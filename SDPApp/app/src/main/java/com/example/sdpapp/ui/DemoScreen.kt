package com.example.sdpapp.ui

import android.os.Build
import androidx.annotation.RequiresApi
import androidx.compose.foundation.layout.Box
import androidx.compose.foundation.layout.Column
import androidx.compose.foundation.layout.fillMaxSize
import androidx.compose.foundation.layout.fillMaxWidth
import androidx.compose.foundation.layout.height
import androidx.compose.foundation.layout.padding
import androidx.compose.material3.Button
import androidx.compose.material3.ButtonDefaults
import androidx.compose.material3.MaterialTheme
import androidx.compose.material3.Text
import androidx.compose.runtime.Composable
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.platform.LocalContext
import androidx.compose.ui.text.style.TextAlign
import androidx.compose.ui.unit.dp
import androidx.compose.ui.unit.sp
import androidx.navigation.NavController
import com.example.sdpapp.MainActivity
import com.example.sdpapp.bt.RangerBluetoothService

@RequiresApi(Build.VERSION_CODES.TIRAMISU)
@Composable
fun DemoScreen(navController: NavController){
    val context = LocalContext.current
    Column(
        modifier = Modifier.fillMaxSize()
    ){
        Box(
            modifier = Modifier.fillMaxSize().padding(10.dp),
            contentAlignment = Alignment.BottomCenter
        ) {
            val mainActivity = context as MainActivity
            if (mainActivity.bluetoothService?.getConnectionState() == RangerBluetoothService.STATE_READY) {
                Button(
                    onClick = { runRanger(context) },
                    modifier = Modifier
                        .fillMaxWidth()
                        .height(500.dp)
                        .align(Alignment.Center),
                    colors = ButtonDefaults.buttonColors(
                        containerColor = MaterialTheme.colorScheme.error,
                        contentColor = MaterialTheme.colorScheme.onBackground
                    ),
                ) {
                    Text(
                        "Start Demo",
                        fontSize = 90.sp,
                        color = MaterialTheme.colorScheme.onBackground,
                        lineHeight = 100.sp,
                        textAlign = TextAlign.Center,
                        modifier = Modifier.fillMaxWidth()
                    )
                }
            }
            else{
                Button(
                    onClick = { runRanger(context)
                              },
                    modifier = Modifier
                        .fillMaxWidth()
                        .height(500.dp)
                        .align(Alignment.Center),
                    colors = ButtonDefaults.buttonColors(
                        containerColor = MaterialTheme.colorScheme.error,
                        contentColor = MaterialTheme.colorScheme.onBackground
                    ),
                ) {
                    Text(
                        "Connect to Robot",
                        fontSize = 80.sp,
                        color = MaterialTheme.colorScheme.onBackground,
                        lineHeight = 85.sp,
                        textAlign = TextAlign.Center,
                        modifier = Modifier.fillMaxWidth()
                    )
                }
            }
        }
    }
}
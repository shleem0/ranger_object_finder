package com.example.sdpapp.ui

import android.Manifest
import android.content.Context
import android.content.pm.PackageManager
import android.os.Build
import android.util.Log
import android.widget.Toast
import androidx.annotation.RequiresApi
import androidx.compose.foundation.BorderStroke
import androidx.compose.foundation.background
import androidx.compose.foundation.border
import androidx.compose.foundation.layout.Box
import androidx.compose.foundation.layout.Column
import androidx.compose.foundation.layout.Spacer
import androidx.compose.foundation.layout.fillMaxSize
import androidx.compose.foundation.layout.fillMaxWidth
import androidx.compose.foundation.layout.height
import androidx.compose.foundation.layout.padding
import androidx.compose.foundation.lazy.LazyColumn
import androidx.compose.foundation.lazy.items
import androidx.compose.foundation.rememberScrollState
import androidx.compose.foundation.shape.RoundedCornerShape
import androidx.compose.foundation.verticalScroll
import androidx.compose.material.icons.Icons
import androidx.compose.material.icons.filled.ArrowDropDown
import androidx.compose.material3.Button
import androidx.compose.material3.ButtonDefaults
import androidx.compose.material3.DropdownMenu
import androidx.compose.material3.DropdownMenuItem
import androidx.compose.material3.Icon
import androidx.compose.material3.MaterialTheme
import androidx.compose.material3.OutlinedButton
import androidx.compose.material3.Text
import androidx.compose.material3.TextButton
import androidx.compose.runtime.Composable
import androidx.compose.runtime.getValue
import androidx.compose.runtime.mutableStateOf
import androidx.compose.runtime.remember
import androidx.compose.runtime.setValue
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.platform.LocalContext
import androidx.compose.ui.text.font.FontWeight
import androidx.compose.ui.unit.dp
import androidx.compose.ui.unit.sp
import androidx.core.content.ContextCompat
import androidx.navigation.NavController
import com.example.sdpapp.MainActivity
import com.example.sdpapp.PermissionManager
import com.example.sdpapp.bt.RangerBluetoothService

@RequiresApi(Build.VERSION_CODES.TIRAMISU)
@Composable
fun SearchScreen(navController: NavController) {
    var expanded by remember { mutableStateOf(false) }
    var selectedOption by remember { mutableStateOf("Select an Item") }
    val context = LocalContext.current

    val options = getItemNames(context)

    Column(
        modifier = Modifier
            .fillMaxSize()
            .background(MaterialTheme.colorScheme.background)
            .padding(horizontal = 10.dp)
    ) {
        TextButton(
            onClick = { navController.navigate("home") }
        ) {
            Text(
                "< Back",
                color = MaterialTheme.colorScheme.surfaceBright,
                fontSize = 18.sp
            )
        }

        Text(
            text = "Find Item",
            style = MaterialTheme.typography.bodyLarge,
            color = MaterialTheme.colorScheme.surfaceBright
        )

        Text(
            text = "Choose Item",
            color = MaterialTheme.colorScheme.surfaceBright,
            fontSize = 30.sp,
            fontWeight = FontWeight.Bold,
            modifier = Modifier.padding(vertical = 10.dp)
        )

        Column(modifier = Modifier.fillMaxWidth()) {
            OutlinedButton(
                onClick = { expanded = true },
                modifier = Modifier
                    .fillMaxWidth()
                    .height(60.dp)
            ) {
                Text(
                    text = selectedOption,
                    fontSize = 22.sp,
                    color = MaterialTheme.colorScheme.secondary
                )
                Icon(Icons.Filled.ArrowDropDown, contentDescription = "Dropdown Arrow")
            }

            DropdownMenu(
                expanded = expanded,
                onDismissRequest = { expanded = false }
            ) {
                options.forEach { option ->
                    val formattedOption = option.replaceFirstChar { it.uppercase() }
                    DropdownMenuItem(
                        text = {
                            Text(
                                formattedOption,
                                fontSize = 25.sp
                            )
                        },
                        onClick = {
                            selectedOption = formattedOption
                            expanded = false
                        }
                    )
                }
            }
        }

        LazyColumn(
        ) {
            item {
                Text(
                    text = "Remember:",
                    color = MaterialTheme.colorScheme.surfaceBright,
                    fontSize = 30.sp,
                    fontWeight = FontWeight.Bold,
                    modifier = Modifier.padding(vertical = 10.dp)
                )
            }
            items(
                listOf(
                    " - The robot can take up to n minutes to find an item.",
                    " - The robot will only be able to find the item in an enclosed area.",
                    " - The robot will grab the item, so it is not suitable for fragile objects.",
                    " - The robot will be sent the photos of the item for processing"
                )
            ) { text ->
                Text(
                    text = text,
                    color = MaterialTheme.colorScheme.secondary,
                    fontSize = 18.sp,
                    lineHeight = 22.sp
                )
                Spacer(Modifier.padding(bottom = 5.dp))
            }
        }
    }
    Box(
        modifier = Modifier
            .fillMaxSize().padding(16.dp),
        contentAlignment = Alignment.BottomEnd
    ) {
        Box(
            contentAlignment = Alignment.TopEnd
        ) {
            val mainActivity = context as MainActivity
            if (mainActivity.bluetoothService?.getConnectionState() == RangerBluetoothService.STATE_READY) {
                if (options.contains(selectedOption)) {
                    Button(
                        onClick = {
                            runRanger(navController, context, selectedOption)
                            navController.navigate("home")
                        },
                        modifier = Modifier
                            .height(70.dp)
                            .fillMaxWidth()
                            .border(
                                BorderStroke(12.dp, MaterialTheme.colorScheme.secondary),
                                shape = RoundedCornerShape(16.dp)
                            ),
                        colors = ButtonDefaults.buttonColors(
                            containerColor = MaterialTheme.colorScheme.secondary,
                            contentColor = MaterialTheme.colorScheme.onBackground
                        )
                    ) {
                        Text(
                            "Find Item",
                            fontSize = 34.sp,
                            color = MaterialTheme.colorScheme.onBackground
                        )
                    }
                }
                else {
                    Button(
                        onClick = {
                            Toast.makeText(context,
                                "Please select an item first.",
                                Toast.LENGTH_SHORT).show()
                        },
                        modifier = Modifier
                            .height(70.dp)
                            .fillMaxWidth()
                            .border(
                                BorderStroke(12.dp, MaterialTheme.colorScheme.secondary),
                                shape = RoundedCornerShape(16.dp)
                            ),
                        colors = ButtonDefaults.buttonColors(
                            containerColor = MaterialTheme.colorScheme.secondary,
                            contentColor = MaterialTheme.colorScheme.onBackground
                        )
                    ) {
                        Text(
                            "Find Item",
                            fontSize = 34.sp,
                            color = MaterialTheme.colorScheme.onBackground
                        )
                    }
                }
            }
            else{
                Button(
                    onClick = {
                        runRanger(navController, context, selectedOption)
                    },
                    modifier = Modifier
                        .height(70.dp)
                        .fillMaxWidth()
                        .border(
                            BorderStroke(12.dp, MaterialTheme.colorScheme.secondary),
                            shape = RoundedCornerShape(16.dp)
                        ),
                    colors = ButtonDefaults.buttonColors(
                        containerColor = MaterialTheme.colorScheme.secondary,
                        contentColor = MaterialTheme.colorScheme.onBackground
                    )
                ) {
                    Text(
                        "Connect to Robot",
                        fontSize = 34.sp,
                        color = MaterialTheme.colorScheme.onBackground
                    )
                }
            }
        }
    }
}

@RequiresApi(Build.VERSION_CODES.TIRAMISU)
fun runRanger(navController: NavController, context: Context, selectedOption: String) {
    Log.i("DemoScreen", "Start demooo")
    val mainActivity = context as MainActivity
    val permissionManager = context as PermissionManager

    if (ContextCompat.checkSelfPermission(context, Manifest.permission.BLUETOOTH_CONNECT) != PackageManager.PERMISSION_GRANTED) {
        Toast.makeText(context,
            "Please wait while connecting.",
            Toast.LENGTH_SHORT).show()
        permissionManager.requestBluetoothPermission()
        Log.i("DemoScreen", "Start demo0")

        return
    }

    if (mainActivity.bluetoothService?.getConnectionState() == RangerBluetoothService.STATE_READY) {
        Log.i("DemoScreen", "Start demoo1")
        mainActivity.bluetoothService?.startDemo(selectedOption)

    } else {
        mainActivity.registerReceiverSafely()
        Log.i("DemoScreen", "Start demooo2")

        val s = mainActivity.bluetoothService
        if (s == null) {
            Toast.makeText(context,
                "Please try again",
                Toast.LENGTH_SHORT).show()
            Log.w("DemoScreen", "can't connect, no service")
        } else {
            Toast.makeText(context,
                "Please wait while connecting.",
                Toast.LENGTH_SHORT).show()
            s.connectForDemo()
        }
    }
}
@file:OptIn(ExperimentalMaterial3Api::class)

package com.example.sdpapp.ui

import android.Manifest
import android.content.Context
import android.content.pm.PackageManager
import android.os.Build
import android.util.Log
import android.widget.Toast
import androidx.annotation.RequiresApi
import androidx.compose.foundation.BorderStroke
import androidx.compose.foundation.Image
import androidx.compose.foundation.background
import androidx.compose.foundation.border
import androidx.compose.foundation.gestures.detectTapGestures
import androidx.compose.foundation.layout.Arrangement
import androidx.compose.foundation.layout.Box
import androidx.compose.foundation.layout.Column
import androidx.compose.foundation.layout.Row
import androidx.compose.foundation.layout.Spacer
import androidx.compose.foundation.layout.fillMaxSize
import androidx.compose.foundation.layout.fillMaxWidth
import androidx.compose.foundation.layout.height
import androidx.compose.foundation.layout.padding
import androidx.compose.foundation.layout.size
import androidx.compose.foundation.layout.width
import androidx.compose.foundation.lazy.LazyColumn
import androidx.compose.foundation.lazy.items
import androidx.compose.foundation.shape.CircleShape
import androidx.compose.foundation.shape.RoundedCornerShape
import androidx.compose.foundation.text.BasicTextField
import androidx.compose.material.icons.Icons
import androidx.compose.material.icons.filled.Add
import androidx.compose.material.icons.filled.*
import androidx.compose.material.icons.filled.Notifications
import androidx.compose.material3.AlertDialog
import androidx.compose.material3.Button
import androidx.compose.material3.ButtonDefaults
import androidx.compose.material3.ExperimentalMaterial3Api
import androidx.compose.material3.Text
import androidx.compose.runtime.Composable
import androidx.compose.ui.Modifier
import androidx.navigation.NavController
import androidx.compose.material3.ExtendedFloatingActionButton
import androidx.compose.material3.Icon
import androidx.compose.material3.IconButton
import androidx.compose.material3.MaterialTheme
import androidx.compose.material3.TextButton
import androidx.compose.material3.contentColorFor
import androidx.compose.runtime.getValue
import androidx.compose.runtime.mutableStateOf
import androidx.compose.runtime.remember
import androidx.compose.runtime.setValue
import androidx.compose.ui.Alignment
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.graphics.vector.ImageVector
import androidx.compose.ui.input.pointer.pointerInput
import androidx.compose.ui.platform.LocalContext
import androidx.compose.ui.platform.LocalFocusManager
import androidx.compose.ui.res.painterResource
import androidx.compose.ui.res.stringResource
import androidx.compose.ui.text.TextStyle
import androidx.compose.ui.text.font.FontWeight
import androidx.compose.ui.text.input.TextFieldValue
import androidx.compose.ui.text.style.TextAlign
import androidx.compose.ui.text.style.TextOverflow
import androidx.compose.ui.tooling.preview.Preview
import androidx.compose.ui.unit.dp
import androidx.compose.ui.unit.sp
import androidx.core.content.ContextCompat
import androidx.navigation.compose.rememberNavController
import com.example.sdpapp.MainActivity
import com.example.sdpapp.PermissionManager
import com.example.sdpapp.R
import com.example.sdpapp.bt.RangerBluetoothService
import com.example.sdpapp.ui.theme.SDPAppTheme
import java.io.File
import java.io.FileOutputStream
import java.io.IOException

@RequiresApi(Build.VERSION_CODES.TIRAMISU)
@Composable
fun HomeScreen(navController: NavController) {
    val context = LocalContext.current
    val itemNames = getItemNames(context)

    Column(
        modifier = Modifier
            .fillMaxSize()
            .background(MaterialTheme.colorScheme.background)
            .padding(20.dp)
    ) {
        Image(
            painter = painterResource(id = R.drawable.rangergreenletters),
            contentDescription = stringResource(id = R.string.logo_description)
        )


        Spacer(modifier = Modifier.padding(vertical = 10.dp))

        Row(modifier = Modifier.fillMaxWidth()) {
            val mainActivity = context as MainActivity
            if (mainActivity.bluetoothService?.getConnectionState() != RangerBluetoothService.STATE_READY) {
                Box(
                    modifier = Modifier
                        .height(100.dp),
                    contentAlignment = Alignment.BottomCenter
                ) {
                    Button(
                        onClick = { connectToRobot(context) },
                        modifier = Modifier
                            .height(100.dp)
                            .width(155.dp)
                            .align(Alignment.BottomEnd)
                            .border(
                                BorderStroke(14.dp, MaterialTheme.colorScheme.secondary),
                                shape = RoundedCornerShape(16.dp)
                            ),
                        colors = ButtonDefaults.buttonColors(
                            containerColor = MaterialTheme.colorScheme.secondary,
                            contentColor = MaterialTheme.colorScheme.onBackground
                        )
                    ) {
                        Text(
                            "Connect to Robot",
                            fontSize = 24.sp,
                            lineHeight = 30.sp,
                            textAlign = TextAlign.Center,
                            color = MaterialTheme.colorScheme.onBackground
                        )
                    }
                }
            } else {
                Box(
                    modifier = Modifier
                        .height(100.dp),
                    contentAlignment = Alignment.BottomCenter
                ) {
                    Button(
                        onClick = { navController.navigate("home") },
                        modifier = Modifier
                            .height(100.dp)
                            .width(155.dp)
                            .align(Alignment.BottomEnd)
                            .border(
                                BorderStroke(3.dp, MaterialTheme.colorScheme.secondary),
                                shape = RoundedCornerShape(16.dp)
                            ),
                        colors = ButtonDefaults.buttonColors(
                            containerColor = MaterialTheme.colorScheme.onBackground,
                            contentColor = MaterialTheme.colorScheme.surfaceBright
                        )
                    ) {
                        Text(
                            "Connected to Robot",
                            fontSize = 22.sp,
                            lineHeight = 28.sp,
                            textAlign = TextAlign.Center,
                            color = MaterialTheme.colorScheme.surfaceBright
                        )
                    }
                }
            }

            Spacer(modifier = Modifier.width(10.dp))

            Box(
                modifier = Modifier
                    .height(100.dp),
                contentAlignment = Alignment.BottomCenter
            ) {
                Button(
                    onClick = { navController.navigate("search") },
                    modifier = Modifier
                        .height(100.dp)
                        .width(155.dp)
                        .align(Alignment.BottomEnd)
                        .border(
                            BorderStroke(14.dp, MaterialTheme.colorScheme.secondary),
                            shape = RoundedCornerShape(16.dp)
                        ),
                    colors = ButtonDefaults.buttonColors(
                        containerColor = MaterialTheme.colorScheme.secondary,
                        contentColor = MaterialTheme.colorScheme.onBackground
                    )
                ) {
                    Text(
                        "Find Item",
                        fontSize = 24.sp,
                        lineHeight = 30.sp,
                        textAlign = TextAlign.Center,
                        color = MaterialTheme.colorScheme.onBackground
                    )
                }
            }
        }

        Spacer(modifier = Modifier.height(10.dp))

        Row(
            horizontalArrangement = Arrangement.SpaceBetween,
            verticalAlignment = Alignment.CenterVertically
        ) {
            Text(
                text = "Items",
                color = MaterialTheme.colorScheme.surfaceBright,
                fontSize = 30.sp,
                fontWeight = FontWeight.Bold,
                modifier = Modifier
                    .padding(vertical = 10.dp)
                    .weight(1f)
            )
            Button(
                onClick = { navController.navigate("addItem") }
            ) {
                Text(
                    "Add Item",
                    fontSize = 18.sp,
                    color = MaterialTheme.colorScheme.surfaceBright
                )
            }
        }
        LazyColumn(
            Modifier.height(255.dp)
        ) {
            items(itemNames) { itemName ->
                val context = LocalContext.current

                val savedIconName = remember { readIconName(context, itemName) }
                val savedIconResId = savedIconName?.let {
                    context.resources.getIdentifier(it, "drawable", context.packageName)
                } ?: R.drawable.sdppencil

                Box(
                    modifier = Modifier.fillMaxWidth()
                ) {
                    Row(
                        modifier = Modifier
                            .fillMaxWidth()
                            .padding(bottom = 10.dp)
                            .height(60.dp),
                        horizontalArrangement = Arrangement.SpaceBetween,
                        verticalAlignment = Alignment.CenterVertically
                    ) {
                        Row(
                            verticalAlignment = Alignment.CenterVertically
                        ) {
                            Image(
                                painter = painterResource(id = savedIconResId),
                                contentDescription = "Icon for $itemName",
                                modifier = Modifier.size(50.dp)
                            )
                            Spacer(modifier = Modifier.width(15.dp))
                            Text(
                                text = itemName.replaceFirstChar { it.uppercase() },
                                modifier = Modifier
                                    .width(160.dp)
                                    .padding(top = 10.dp),
                                fontSize = 32.sp,
                                color = MaterialTheme.colorScheme.surfaceBright,
                                textAlign = TextAlign.Start,
                                overflow = TextOverflow.Ellipsis,
                                lineHeight = 32.sp,
                                maxLines = 1
                            )
                        }
                        Button(
                            onClick = {
                                navController.navigate("deleteItem/$itemName")
                            },
                            colors = ButtonDefaults.buttonColors(
                                containerColor = MaterialTheme.colorScheme.error
                            )
                        ) {
                            Text("Delete", color = Color.White)
                        }
                    }
                }
            }
        }

        FABWithNotification(
            getAlertsFromSharedPreferences(LocalContext.current).size,
            navController
        )
    }
}

fun readIconName(context: Context, itemName: String): String? {
    val folder = File(context.filesDir, itemName.lowercase())
    val iconFile = File(folder, "icon_name.txt")
    return if (iconFile.exists()) {
        iconFile.readText().trim()
    } else {
        null
    }
}

@Composable
fun DeleteRow(navController: NavController, itemName: String, context: Context) {
    AlertDialog(
        onDismissRequest = { },
        confirmButton = {
            TextButton(
                onClick = {
                    val folder = File(context.filesDir, itemName)
                    if (folder.exists()) {
                        folder.deleteRecursively()
                    }
                    navController.navigate("home")
                }
            ) {
                Text("Delete", color = MaterialTheme.colorScheme.error)
            }
        },
        dismissButton = {
            TextButton(onClick = { navController.navigate("home") }) {
                Text("Cancel")
            }
        },
        title = { Text("Delete Item?") },
        text = { Text("Are you sure you want to delete $itemName? All photos will be lost.") }
    )
}

@Composable
fun FABWithNotification(notificationCount: Int, navController: NavController) {
    Box(
        modifier = Modifier
            .fillMaxSize(),
        contentAlignment = Alignment.BottomEnd
    ) {
        Box(
            contentAlignment = Alignment.TopEnd
        ) {
            ExtendedFloatingActionButton(
                onClick = { navController.navigate("alerts") },
                icon = { Icon(Icons.Filled.Notifications, contentDescription = "Alert Button") },
                text = { Text(text = "Alerts") },
                containerColor = MaterialTheme.colorScheme.onBackground,
                contentColor = MaterialTheme.colorScheme.surfaceBright
            )

            if (notificationCount > 0) {
                Box(
                    modifier = Modifier
                        .size(20.dp)
                        .background(MaterialTheme.colorScheme.error, shape = CircleShape)
                        .align(Alignment.TopEnd)
                )
            }
        }
    }
}

@Composable
fun getItemNames(context: Context): List<String> {
    val subfolderNames = mutableListOf<String>()
    val mainDir = context.filesDir

    mainDir.listFiles()?.forEach { subfolder ->
        if (subfolder.isDirectory) {
            subfolderNames.add(subfolder.name)
        }
    }
    return subfolderNames
}

@Composable
fun AddItem(navController: NavController) {
    var itemName by remember { mutableStateOf("") }
    val context = LocalContext.current
    val focusManager = LocalFocusManager.current

    fun createItemFolderAndSaveData(): Boolean {
        Log.d("AddItem", "createItemFolderAndSaveData() called for item: $itemName")
        if (itemName.isNotEmpty()) {
            val folder = File(context.filesDir, itemName.lowercase())
            if (!folder.exists()) {
                folder.mkdirs()

                val itemDataFile = File(folder, "item_details.txt")
                try {
                    val fileOutputStream = FileOutputStream(itemDataFile)
                    val itemDetails = """
                        Name: ${itemName}
                    """.trimIndent()
                    fileOutputStream.write(itemDetails.toByteArray())
                    fileOutputStream.close()
                    return true
                } catch (e: IOException) {
                    Log.e("AddItem", "Error writing file: ${e.message}")
                    Toast.makeText(context, "Error saving item", Toast.LENGTH_SHORT).show()
                    return false
                }

            } else {
                Toast.makeText(
                    context,
                    "Item already exists",
                    Toast.LENGTH_SHORT
                ).show()
                return false
            }
        } else {
            Toast.makeText(
                context,
                "Item Name cannot be empty",
                Toast.LENGTH_SHORT
            ).show()
            return false
        }
    }
    TextButton(
        onClick = { navController.navigate("home") }
    ) {
        Text(
            "< Back",
            color = MaterialTheme.colorScheme.surfaceBright,
            fontSize = 18.sp,
            modifier = Modifier.padding(0.dp)
        )
    }

    Column(
        modifier = Modifier
            .fillMaxSize()
            .padding(vertical = 36.dp, horizontal = 16.dp)
            .pointerInput(Unit) {
                detectTapGestures(onTap = {
                    focusManager.clearFocus()
                })
            }
    ) {
        Text(
            "Add Item",
            style = MaterialTheme.typography.bodyLarge,
            color = MaterialTheme.colorScheme.tertiary,
            modifier = Modifier.padding(bottom = 10.dp)
        )

        Text(
            "Item Name",
            color = MaterialTheme.colorScheme.surfaceBright,
            fontSize = 27.sp,
            fontWeight = FontWeight.Bold
        )

        Spacer(modifier = Modifier.height(10.dp))

        BasicTextField(
            value = itemName,
            onValueChange = { newValue ->
                itemName = newValue.filter { it.isLetter() || it.isWhitespace() || it.isDigit() }
            },
            textStyle = TextStyle(fontSize = 20.sp, color = MaterialTheme.colorScheme.tertiary),
            modifier = Modifier
                .fillMaxWidth()
                .size(30.dp)
                .background(MaterialTheme.colorScheme.onBackground)
                .border(
                    width = 1.dp,
                    color = MaterialTheme.colorScheme.surfaceBright,
                    RoundedCornerShape(4.dp)
                )
                .padding(4.dp)
        )

        Spacer(modifier = Modifier.height(12.dp))

        LazyColumn() {
            items(
                listOf(
                    " - We recommend taking at least 4 photos of the item from different angles.",
                    " - It's best to take photos of your item right away, so you have them before it goes missing!",
                    " - Removing an item will also remove all of its photos.",
                    " - The robot will be sent the photos of the item for processing.",
                    " - If your item ever looks different from the photos, make sure to update the photos accordingly!"
                )
            ) { text ->
                Text(
                    text = text,
                    color = MaterialTheme.colorScheme.surfaceBright,
                    fontSize = 19.sp,
                    lineHeight = 20.sp
                )
                Spacer(Modifier.padding(bottom = 10.dp))
            }
        }

        Box(
            contentAlignment = Alignment.BottomCenter,
            modifier = Modifier.fillMaxSize()
        ) {
            Button(
                onClick = {
                    val isSuccess = createItemFolderAndSaveData()
                    if (isSuccess) {
                        navController.navigate("iconSelection/${itemName}")
                    } else {
                        navController.navigate("addItem")
                    }
                }
            ) {
                Text(
                    "Save Item",
                    fontSize = 25.sp,
                    color = MaterialTheme.colorScheme.surfaceBright,
                )
            }
        }
    }
}

@RequiresApi(Build.VERSION_CODES.TIRAMISU)
fun connectToRobot(context: Context) {
    val mainActivity = context as MainActivity
    val s = mainActivity.bluetoothService
    val permissionManager = context as PermissionManager

    if (ContextCompat.checkSelfPermission(
            context,
            Manifest.permission.BLUETOOTH_CONNECT
        ) != PackageManager.PERMISSION_GRANTED
    ) {
        Toast.makeText(
            context,
            "Please wait while connecting.",
            Toast.LENGTH_SHORT
        ).show()
        permissionManager.requestBluetoothPermission()

        return
    }

    mainActivity.registerReceiverSafely()

    if (s == null) {
        Toast.makeText(context, "Please try again", Toast.LENGTH_SHORT).show()
        Log.w("HomeScreen", "can't connect, no service")
    } else {
        Toast.makeText(context, "Please wait while connecting.", Toast.LENGTH_SHORT).show()
        s.connectForDemo()
    }
}
@file:OptIn(ExperimentalMaterial3Api::class)

package com.example.sdpapp.ui

import android.content.Context
import android.widget.Toast
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
import androidx.compose.ui.input.pointer.pointerInput
import androidx.compose.ui.platform.LocalContext
import androidx.compose.ui.res.painterResource
import androidx.compose.ui.res.stringResource
import androidx.compose.ui.text.TextStyle
import androidx.compose.ui.text.font.FontWeight
import androidx.compose.ui.text.input.TextFieldValue
import androidx.compose.ui.tooling.preview.Preview
import androidx.compose.ui.unit.dp
import androidx.compose.ui.unit.sp
import androidx.navigation.compose.rememberNavController
import com.example.sdpapp.R
import com.example.sdpapp.ui.theme.SDPAppTheme
import java.io.File
import java.io.FileOutputStream
import java.io.IOException

@Composable
fun HomeScreen(navController: NavController) {
    val context = LocalContext.current
    val itemNames = getItemNames(context)
    var showDeleteButton by remember { mutableStateOf(false) }

    Column(
        modifier = Modifier
            .fillMaxSize()
            .background(MaterialTheme.colorScheme.background)
            .padding(20.dp)
            .pointerInput(Unit) {
                detectTapGestures(
                    onTap = {
                        showDeleteButton = false
                    }
                )
            }
    ) {
        Image(
            painter = painterResource(id = R.drawable.rangergreenletters),
            contentDescription = stringResource(id = R.string.logo_description)
        )


        Spacer(modifier = Modifier.padding(vertical = 15.dp))

        Box(
            modifier = Modifier.fillMaxWidth().height(90.dp),
            contentAlignment = Alignment.BottomCenter
        ) {
            Button(
                onClick = { navController.navigate("search") },
                modifier = Modifier
                    .height(90.dp)
                    .fillMaxWidth()
                    .align(Alignment.BottomEnd)
                    .border(
                        BorderStroke(13.dp, MaterialTheme.colorScheme.secondary),
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

        Spacer(modifier = Modifier.padding(vertical = 15.dp))

        Row(
            horizontalArrangement = Arrangement.SpaceBetween,
            verticalAlignment = Alignment.CenterVertically
        ) {
            Text(
                text = "Items",
                color = MaterialTheme.colorScheme.surfaceBright,
                fontSize = 30.sp,
                fontWeight = FontWeight.Bold,
                modifier = Modifier.padding(vertical = 10.dp).weight(1f)
            )
            Button(
                onClick = { navController.navigate("addItem") }
            ) {
                Text("Add Item",
                    fontSize = 18.sp)
            }
        }
        LazyColumn(
            Modifier.height(255.dp)
        ) {
            items(itemNames) { itemName ->
                val imageName = "sdp$itemName"
                val imageResId = remember {
                    context.resources.getIdentifier(imageName, "drawable", context.packageName)
                }
                val fallbackResId = R.drawable.sdppencil
                val finalImageResId = if (imageResId != 0) imageResId else fallbackResId
                Box(
                    modifier = Modifier
                        .fillMaxWidth()
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
                            modifier = Modifier
                                .pointerInput(Unit) {
                                    detectTapGestures(
                                        onLongPress = {
                                            showDeleteButton = true
                                        }
                                    )
                                },
                            verticalAlignment = Alignment.CenterVertically
                        ) {
                            Image(
                                painter = painterResource(id = finalImageResId),
                                contentDescription = "Icon for $itemName",
                                modifier = Modifier.size(50.dp)
                            )
                            Spacer(modifier = Modifier.width(15.dp))
                            Text(
                                text = itemName.replaceFirstChar { it.uppercase() },
                                fontSize = 30.sp,
                                color = MaterialTheme.colorScheme.surfaceBright
                            )
                        }

                        if (showDeleteButton) {
                            Button(
                                onClick = {
                                    navController.navigate("deleteItem/$itemName")
                                    showDeleteButton = false
                                },
                                colors = ButtonDefaults.buttonColors(containerColor = Color.Red)
                            ) {
                                Text("Delete", color = Color.White)
                            }
                        } else {
                            IconButton(
                                onClick = { navController.navigate("camera/$itemName") }
                            ) {
                                Icon(
                                    Icons.Filled.Add,
                                    contentDescription = "Add " + itemName + " Photos",
                                    modifier = Modifier.size(35.dp),
                                    contentColorFor(MaterialTheme.colorScheme.surfaceBright)
                                )
                            }
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

@Composable
fun DeleteRow(navController: NavController, itemName: String, context: Context){
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
                Text("Delete", color = Color.Red)
            } },
        dismissButton = { TextButton(onClick = { navController.navigate("home") }) {
                    Text("Cancel") } },
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
                        .background(Color.Red, shape = CircleShape)
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
fun AddItem(navController: NavController){
    var itemName by remember { mutableStateOf(TextFieldValue()) }
    var additionalDetails by remember { mutableStateOf(TextFieldValue()) }
    val context = LocalContext.current

    fun createItemFolderAndSaveData() {
        if (itemName.text.isNotEmpty()) {
            val folder = File(context.filesDir, itemName.text.lowercase())
            if (!folder.exists()) {
                folder.mkdirs()

                val itemDataFile = File(folder, "item_details.txt")
                try {
                    val fileOutputStream = FileOutputStream(itemDataFile)
                    val itemDetails = """
                        Name: ${itemName.text}
                        Icon: R.drawable.sdppencil
                        Details: ${additionalDetails.text}
                    """.trimIndent()

                    fileOutputStream.write(itemDetails.toByteArray())
                    fileOutputStream.close()

                    Toast.makeText(context,
                        "Item folder created",
                        Toast.LENGTH_SHORT).show()
                } catch (e: IOException) {
                    e.printStackTrace()
                }
            } else {
                Toast.makeText(context,
                    "Item already exists",
                    Toast.LENGTH_SHORT).show()
            }
        } else {
            Toast.makeText(context,
                "Item Name cannot be empty",
                Toast.LENGTH_SHORT).show()
        }
    }

    Column(
        modifier = Modifier
            .fillMaxSize()
            .padding(16.dp),
    ) {
        TextButton(
            onClick = { navController.navigate("home") }
        ) {
            Text(
                "< back",
                color = MaterialTheme.colorScheme.surfaceBright,
                fontSize = 18.sp,
                modifier = Modifier.padding(0.dp)
            )
        }
        Text("Add Item",
            style = MaterialTheme.typography.bodyLarge,
            color = MaterialTheme.colorScheme.tertiary,
            modifier = Modifier.padding(bottom = 10.dp))
        Text("Item Name",
            color = MaterialTheme.colorScheme.surfaceBright,
            fontSize = 27.sp,
            fontWeight = FontWeight.Bold
        )
        BasicTextField(
            value = itemName,
            onValueChange = { itemName = it },
            textStyle = TextStyle(fontSize = 20.sp),
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

        Spacer(modifier = Modifier.height(16.dp))

        Text("Short Description",
            color = MaterialTheme.colorScheme.surfaceBright,
            fontSize = 27.sp,
            fontWeight = FontWeight.Bold
        )
        BasicTextField(
            value = additionalDetails,
            onValueChange = { additionalDetails = it },
            textStyle = TextStyle(fontSize = 20.sp),
            modifier = Modifier
                .fillMaxWidth()
                .size(180.dp)
                .background(MaterialTheme.colorScheme.onBackground)
                .border(
                    width = 1.dp,
                    color = MaterialTheme.colorScheme.surfaceBright,
                    RoundedCornerShape(4.dp)
                )
                .padding(4.dp)
        )

        Spacer(modifier = Modifier.height(24.dp))

        Text("We recommend taking at least 3 photos of the item from different angles.",
            color = MaterialTheme.colorScheme.surfaceBright,
            fontSize = 17.sp,
            lineHeight = 20.sp
        )

        Box(
            contentAlignment = Alignment.BottomCenter,
            modifier = Modifier.fillMaxSize()
        ) {
            Button(
                onClick = { createItemFolderAndSaveData()
                          navController.navigate("home")
                          },
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

@Preview(showBackground = true)
@Composable
fun HomeScreenPreview() {
    SDPAppTheme {
        HomeScreen(navController = rememberNavController())
    }
}
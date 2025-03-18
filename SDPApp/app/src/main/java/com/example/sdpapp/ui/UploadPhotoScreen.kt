@file:OptIn(ExperimentalPermissionsApi::class)

package com.example.sdpapp.ui

import android.Manifest
import android.content.Context
import android.net.Uri
import android.provider.Settings
import android.content.Intent
import android.content.pm.PackageManager
import android.os.Build
import android.util.Log
import androidx.activity.compose.rememberLauncherForActivityResult
import androidx.activity.result.contract.ActivityResultContracts
import androidx.compose.foundation.BorderStroke
import androidx.compose.foundation.Image
import androidx.compose.foundation.border
import androidx.compose.foundation.clickable
import androidx.compose.foundation.layout.*
import androidx.compose.foundation.lazy.grid.GridCells
import androidx.compose.foundation.lazy.grid.LazyVerticalGrid
import androidx.compose.foundation.lazy.grid.items
import androidx.compose.foundation.shape.RoundedCornerShape
import androidx.compose.material.icons.Icons
import androidx.compose.material.icons.filled.Clear
import androidx.compose.material3.*
import androidx.compose.runtime.*
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.graphics.painter.Painter
import androidx.compose.ui.platform.LocalContext
import androidx.compose.ui.unit.dp
import androidx.compose.ui.unit.sp
import androidx.compose.ui.res.painterResource
import androidx.compose.ui.text.style.TextAlign
import androidx.core.content.ContextCompat
import androidx.navigation.NavController
import coil.compose.rememberAsyncImagePainter
import com.example.sdpapp.R
import com.google.accompanist.permissions.*
import java.io.File

@Composable
fun UploadScreen(navController: NavController, itemName: String) {
    val context = LocalContext.current

    val permissionLauncher = rememberLauncherForActivityResult(
        contract = ActivityResultContracts.RequestPermission()
    ) { isGranted ->
        if (isGranted) {
            navController.navigate("uploadContent/$itemName")
        } else {
            navController.navigate("deniedPermission")
        }
    }

    LaunchedEffect(Unit) {
        if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.TIRAMISU) {
            permissionLauncher.launch(Manifest.permission.READ_MEDIA_IMAGES)
        } else {
            permissionLauncher.launch(Manifest.permission.READ_EXTERNAL_STORAGE)
        }
    }

    val permissionGranted = if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.TIRAMISU) {
        ContextCompat.checkSelfPermission(
            context,
            Manifest.permission.READ_MEDIA_IMAGES
        ) == PackageManager.PERMISSION_GRANTED
    } else {
        ContextCompat.checkSelfPermission(
            context,
            Manifest.permission.READ_EXTERNAL_STORAGE
        ) == PackageManager.PERMISSION_GRANTED
    }

    if (permissionGranted) {
        UploadContent(navController, itemName)
    } else {
        PermissionDeniedView(context)
    }
}

@Composable
fun PermissionDeniedView(context: android.content.Context) {
    Column(
        modifier = Modifier
            .fillMaxWidth()
            .padding(16.dp),
        horizontalAlignment = Alignment.CenterHorizontally
    ) {
        Text(
            text = "File access permission is required to upload images.",
            fontSize = 18.sp,
            color = MaterialTheme.colorScheme.error
        )
        Spacer(modifier = Modifier.height(16.dp))
        Button(
            onClick = {
                val intent = Intent(Settings.ACTION_APPLICATION_DETAILS_SETTINGS).apply {
                    data = android.net.Uri.parse("package:${context.packageName}")
                }
                context.startActivity(intent)
            }
        ) {
            Text("Open Settings")
        }
    }
}

@Composable
fun UploadContent(navController: NavController, itemName: String) {
    val context = LocalContext.current
    var selectedImages by remember { mutableStateOf<List<Uri>>(emptyList()) }

    val imagePickerLauncher = rememberLauncherForActivityResult(
        contract = ActivityResultContracts.GetMultipleContents()
    ) { uris ->
        selectedImages = if (uris.size > 5) uris.take(5) else uris
    }

    Box(
        modifier = Modifier
            .fillMaxSize()
            .padding(16.dp)
    ) {
        Column(
            modifier = Modifier
                .fillMaxSize()
                .padding(bottom = 80.dp)
        ) {
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
            Button(
                onClick = { imagePickerLauncher.launch("image/*") },
                modifier = Modifier
                    .height(75.dp)
                    .fillMaxWidth()
                    .padding(vertical = 8.dp)
                    .border(
                        BorderStroke(3.dp, MaterialTheme.colorScheme.secondary),
                        shape = RoundedCornerShape(16.dp)
                    ),
                colors = ButtonDefaults.buttonColors(
                    containerColor = MaterialTheme.colorScheme.onBackground,
                    contentColor = MaterialTheme.colorScheme.secondary
                )
            ) {
                Text("Select Images (Max 5)",
                    fontSize = 24.sp,
                    lineHeight = 30.sp,
                    textAlign = TextAlign.Center,
                    color = MaterialTheme.colorScheme.secondary
                )
            }

            Spacer(modifier = Modifier.height(16.dp))

            if (selectedImages.isNotEmpty()) {
                LazyVerticalGrid(
                    columns = GridCells.Fixed(3),
                    modifier = Modifier
                        .fillMaxWidth()
                        .height(300.dp * selectedImages.size)
                ) {
                    items(selectedImages) { uri ->
                        ImagePreview(uri) {
                            selectedImages = selectedImages - uri
                        }
                    }
                }
            }
        }

        Column(
            modifier = Modifier
                .align(Alignment.BottomCenter)
                .padding(bottom = 16.dp),
            horizontalAlignment = Alignment.CenterHorizontally
        ) {
            Text(
                text = "Maximum 5 images allowed per upload.",
                color = MaterialTheme.colorScheme.surfaceBright,
                fontSize = 17.sp,
                lineHeight = 22.sp,
                textAlign = TextAlign.Center
            )
            Spacer(Modifier.padding(bottom = 8.dp))
            Text(
                text = "Images will be compressed in size and cropped to be square.",
                color = MaterialTheme.colorScheme.surfaceBright,
                fontSize = 17.sp,
                lineHeight = 22.sp,
                textAlign = TextAlign.Center
            )
            Spacer(Modifier.padding(bottom = 12.dp))

            Button(
                onClick = {
                    saveSelectedImages(context, selectedImages, itemName)
                    navController.navigate("home")
                },
                enabled = selectedImages.isNotEmpty(),
                modifier = Modifier
                    .fillMaxWidth()
                    .border(
                        BorderStroke(4.dp, MaterialTheme.colorScheme.surfaceBright),
                        shape = RoundedCornerShape(16.dp)
                    ),
                colors = ButtonDefaults.buttonColors(
                    containerColor = MaterialTheme.colorScheme.surfaceBright,
                    contentColor = MaterialTheme.colorScheme.onBackground
                )
            ) {
                Text(
                    text = "Confirm Upload",
                    fontSize = 24.sp,
                    lineHeight = 30.sp,
                    textAlign = TextAlign.Center,
                    color = MaterialTheme.colorScheme.onBackground
                )
            }
        }
    }
}

@Composable
fun ImagePreview(uri: Uri, onRemove: () -> Unit) {
    Box(
        modifier = Modifier
            .padding(4.dp)
            .size(130.dp)
            .clickable { onRemove() },
        contentAlignment = Alignment.Center
    ) {
        Image(
            painter = rememberAsyncImagePainter(uri),
            contentDescription = "Selected image",
            modifier = Modifier
                .size(130.dp)
        )
    }
}

private fun saveSelectedImages(context: Context, uris: List<Uri>, itemName: String) {
    val directory = File(context.filesDir, itemName)
    if (!directory.exists()) {
        directory.mkdirs()
    }

    uris.forEach { uri ->
        try {
            val inputStream = context.contentResolver.openInputStream(uri)
            val originalFile = File(directory, "IMG_${System.currentTimeMillis()}.jpg")
            inputStream?.use { input ->
                originalFile.outputStream().use { output ->
                    input.copyTo(output)
                }
            }

            val compressedFile = compressImage(context, originalFile)

            if (compressedFile.exists()) {
                Log.d("FileSave", "Compressed image saved at: ${compressedFile.absolutePath}")

                if (originalFile.exists()) {
                    originalFile.delete()
                    Log.d("FileCleanup", "Deleted original file: ${originalFile.absolutePath}")
                }
            } else {
                Log.e("FileSave", "Failed to save compressed image")
            }
        } catch (e: Exception) {
            Log.e("FileSave", "Error saving image: ${e.message}")
        }
    }
}
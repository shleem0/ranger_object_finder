@file:OptIn(ExperimentalPermissionsApi::class)

package com.example.sdpapp.ui

import android.Manifest
import androidx.compose.foundation.Image
import android.content.Context
import android.content.Intent
import android.net.Uri
import android.os.Environment
import android.provider.Settings
import android.util.Log
import androidx.camera.core.CameraSelector
import androidx.camera.core.ImageCapture
import androidx.camera.core.ImageCaptureException
import androidx.camera.core.Preview
import androidx.camera.lifecycle.ProcessCameraProvider
import androidx.camera.view.PreviewView
import androidx.compose.foundation.layout.Arrangement
import androidx.compose.foundation.layout.Box
import androidx.compose.foundation.layout.Column
import androidx.compose.foundation.layout.Row
import androidx.compose.foundation.layout.Spacer
import androidx.compose.foundation.layout.fillMaxSize
import androidx.compose.foundation.layout.fillMaxWidth
import androidx.compose.foundation.layout.height
import androidx.compose.foundation.layout.padding
import androidx.compose.material3.Button
import androidx.compose.material3.ButtonColors
import androidx.compose.material3.MaterialTheme
import androidx.compose.material3.Text
import androidx.compose.material3.TextButton
import androidx.compose.runtime.Composable
import androidx.compose.runtime.LaunchedEffect
import androidx.compose.runtime.getValue
import androidx.compose.runtime.mutableStateOf
import androidx.compose.runtime.remember
import androidx.compose.runtime.setValue
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.platform.LocalContext
import androidx.compose.ui.text.style.TextAlign
import androidx.compose.ui.unit.dp
import androidx.compose.ui.unit.sp
import androidx.compose.ui.viewinterop.AndroidView
import androidx.core.content.ContextCompat
import androidx.core.content.FileProvider
import androidx.lifecycle.compose.LocalLifecycleOwner
import androidx.navigation.NavController
import coil.compose.rememberAsyncImagePainter
import com.google.accompanist.permissions.ExperimentalPermissionsApi
import com.google.accompanist.permissions.isGranted
import com.google.accompanist.permissions.rememberPermissionState
import java.io.File
import java.text.SimpleDateFormat
import java.util.Locale

@Composable
fun CameraScreen(navController: NavController, name: String) {
    TextButton(
        onClick = { navController.navigate("home") },
        colors = ButtonColors(
            MaterialTheme.colorScheme.background,
            MaterialTheme.colorScheme.background,
            MaterialTheme.colorScheme.background,
            MaterialTheme.colorScheme.background
        )
    ) {
        Text(
            "< Back",
            color = MaterialTheme.colorScheme.surfaceBright,
            fontSize = 18.sp,
            modifier = Modifier.padding(bottom = 40.dp)
        )
    }
    CheckCameraPermission {}
    CameraPreview(navController, name)
}

@Composable
fun CheckCameraPermission(onPermissionGranted: () -> Unit) {
    val cameraPermissionState = rememberPermissionState(Manifest.permission.CAMERA)
    val context = LocalContext.current
    LaunchedEffect(Unit) {
        cameraPermissionState.launchPermissionRequest()
    }

    if (cameraPermissionState.status.isGranted) {
        onPermissionGranted()
    }
    else {
        Column (
            modifier = Modifier.fillMaxWidth()
        ) {
            Spacer(modifier = Modifier.padding(top = 30.dp))
            Text(
                text = "Camera permission is required to use this feature.",
                color = MaterialTheme.colorScheme.surfaceBright,
                fontSize = 20.sp,
                modifier = Modifier.padding(horizontal = 12.dp),
                lineHeight = 22.sp
            )
            Button(
                modifier = Modifier.padding(4.dp),
                onClick = {
                    val intent = Intent(Settings.ACTION_APPLICATION_DETAILS_SETTINGS).apply {
                        data = Uri.parse("package:${context.packageName}")
                    }
                    context.startActivity(intent)
                }) {
                Text(
                    text = "Go to Settings",
                    color = MaterialTheme.colorScheme.surfaceBright,
                    lineHeight = 22.sp,
                    fontSize = 20.sp
                )
            }
        }
    }
}

@Composable
fun CameraPreview(navController: NavController, name: String) {
    val context = LocalContext.current
    val lifecycleOwner = LocalLifecycleOwner.current
    val cameraProviderFuture = remember { ProcessCameraProvider.getInstance(context) }

    val imageCapture = remember {
        ImageCapture.Builder()
            .setCaptureMode(ImageCapture.CAPTURE_MODE_MAXIMIZE_QUALITY)
            .build()
    }

    val previewView = remember { PreviewView(context) }

    var capturedImageUri by remember { mutableStateOf<Uri?>(null) }
    var capturedFile by remember { mutableStateOf<File?>(null) }

    Column (
        modifier = Modifier.fillMaxWidth()
    ) {
        Spacer(modifier = Modifier.padding(top = 30.dp))
        if (capturedImageUri == null) {
            Text(
                text = "Make sure the area is free of clutter and only shows the item.",
                color = MaterialTheme.colorScheme.secondary,
                fontSize = 21.sp,
                modifier = Modifier.padding(6.dp),
                lineHeight = 22.sp,
                textAlign = TextAlign.Center
            )
        }
        else {
            Text(
                "All images are saved locally on your phone.",
                color = MaterialTheme.colorScheme.secondary,
                fontSize = 21.sp,
                modifier = Modifier.padding(6.dp),
                lineHeight = 22.sp,
                textAlign = TextAlign.Center
            )
        }
        Spacer(modifier = Modifier.padding(bottom = 10.dp))
        LaunchedEffect(Unit) {
            val executor = ContextCompat.getMainExecutor(context)
            cameraProviderFuture.addListener({
                val cameraProvider = cameraProviderFuture.get()
                val preview = Preview.Builder().build().also {
                    it.setSurfaceProvider(previewView.surfaceProvider)
                }

                val cameraSelector = CameraSelector.Builder()
                    .requireLensFacing(CameraSelector.LENS_FACING_BACK)
                    .build()

                try {
                    cameraProvider.unbindAll()
                    cameraProvider.bindToLifecycle(
                        lifecycleOwner,
                        cameraSelector,
                        preview,
                        imageCapture
                    )
                } catch (exc: Exception) {
                    Log.e("CameraPreview", "Use case binding failed", exc)
                }
            }, executor)
        }

        Box(
            modifier = Modifier
                .fillMaxWidth()
                .height(600.dp),
            contentAlignment = Alignment.TopCenter
        ) {
            if (capturedImageUri == null) {
                AndroidView(
                    factory = { previewView },
                    modifier = Modifier
                        .fillMaxWidth()
                        .height(500.dp)
                )
            } else {
                Image(
                    painter = rememberAsyncImagePainter(capturedImageUri),
                    contentDescription = "Captured photo",
                    modifier = Modifier
                        .fillMaxWidth()
                        .height(450.dp)
                )
            }

            Column(
                modifier = Modifier.align(Alignment.BottomCenter)
            ) {
                if (capturedImageUri == null) {
                    Button(
                        onClick = {
                            captureImage(context, imageCapture, name) { uri, file ->
                                capturedImageUri = uri
                                capturedFile = file
                            }
                        },
                        modifier = Modifier.padding(16.dp),
                        colors = ButtonColors(
                            containerColor = MaterialTheme.colorScheme.secondary,
                            contentColor = MaterialTheme.colorScheme.onBackground,
                            disabledContainerColor = MaterialTheme.colorScheme.secondary,
                            disabledContentColor = MaterialTheme.colorScheme.onBackground
                        )
                    ) {
                        Text("Capture Image")
                    }
                } else {
                    Row(modifier = Modifier.padding(16.dp),
                        horizontalArrangement = Arrangement.Absolute.SpaceBetween
                        ) {
                        Button(
                            onClick = {
                                capturedFile?.delete()
                                capturedImageUri = null
                                capturedFile = null
                            },
                            modifier = Modifier.padding(8.dp).align(Alignment.Top),
                            colors = ButtonColors(
                                containerColor = MaterialTheme.colorScheme.secondary,
                                contentColor = MaterialTheme.colorScheme.onBackground,
                                disabledContainerColor = MaterialTheme.colorScheme.secondary,
                                disabledContentColor = MaterialTheme.colorScheme.onBackground
                            )
                        ) {
                            Text("Retake",
                                fontSize = 18.sp,
                                color = MaterialTheme.colorScheme.onBackground
                            )
                        }
                        Button(
                            onClick = {
                                navController.navigate("photos")
                            },
                            modifier = Modifier.padding(8.dp).align(Alignment.Bottom),
                            colors = ButtonColors(
                                containerColor = MaterialTheme.colorScheme.secondary,
                                contentColor = MaterialTheme.colorScheme.onBackground,
                                disabledContainerColor = MaterialTheme.colorScheme.secondary,
                                disabledContentColor = MaterialTheme.colorScheme.onBackground
                            )
                        ) {
                            Text("Save",
                                fontSize = 18.sp,
                                color = MaterialTheme.colorScheme.onBackground
                            )
                        }
                    }
                }
            }
        }
    }
}

private fun captureImage(context: Context, imageCapture: ImageCapture,
                         name: String, onImageCaptured: (Uri, File) -> Unit) {
    val directory = File(context.filesDir, name)
    if (!directory.exists()) {
        directory.mkdirs()
    }

    val photoFile = File(
        directory,
        "IMG_${SimpleDateFormat("yyyyMMdd_HHmmss", Locale.UK).format(System.currentTimeMillis())}.jpg"
    )

    val outputOptions = ImageCapture.OutputFileOptions.Builder(photoFile).build()

    imageCapture.takePicture(
        outputOptions,
        ContextCompat.getMainExecutor(context),
        object : ImageCapture.OnImageSavedCallback {
            override fun onImageSaved(output: ImageCapture.OutputFileResults) {
                val savedUri: Uri = FileProvider.getUriForFile(
                    context,
                    "${context.packageName}.provider",
                    photoFile
                )
                Log.d("CameraPreview", "Photo saved at: $savedUri")

                onImageCaptured(savedUri, photoFile)
            }

            override fun onError(exception: ImageCaptureException) {
                Log.e("CameraPreview", "Image capture failed", exception)
            }
        }
    )
}

package com.example.sdpapp.ui

import android.content.Context
import androidx.compose.foundation.Image
import androidx.compose.foundation.background
import androidx.compose.foundation.clickable
import androidx.compose.foundation.layout.Arrangement
import androidx.compose.foundation.layout.Box
import androidx.compose.foundation.layout.Column
import androidx.compose.foundation.layout.PaddingValues
import androidx.compose.foundation.layout.Row
import androidx.compose.foundation.layout.Spacer
import androidx.compose.foundation.layout.fillMaxSize
import androidx.compose.foundation.layout.fillMaxWidth
import androidx.compose.foundation.layout.height
import androidx.compose.foundation.layout.padding
import androidx.compose.foundation.layout.size
import androidx.compose.foundation.layout.width
import androidx.compose.foundation.lazy.LazyColumn
import androidx.compose.foundation.lazy.LazyRow
import androidx.compose.foundation.lazy.items
import androidx.compose.foundation.shape.RoundedCornerShape
import androidx.compose.material.icons.Icons
import androidx.compose.material.icons.filled.Add
import androidx.compose.material.icons.filled.ArrowDropDown
import androidx.compose.material3.Button
import androidx.compose.material3.ButtonDefaults
import androidx.compose.material3.DropdownMenu
import androidx.compose.material3.DropdownMenuItem
import androidx.compose.material3.HorizontalDivider
import androidx.compose.material3.Icon
import androidx.compose.material3.IconButton
import androidx.compose.material3.MaterialTheme
import androidx.compose.material3.OutlinedButton
import androidx.compose.material3.Text
import androidx.compose.material3.contentColorFor
import androidx.compose.runtime.Composable
import androidx.compose.runtime.getValue
import androidx.compose.runtime.mutableStateOf
import androidx.compose.runtime.remember
import androidx.compose.runtime.setValue
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.draw.clip
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.layout.ContentScale
import androidx.compose.ui.platform.LocalContext
import androidx.compose.ui.res.stringResource
import androidx.compose.ui.text.font.FontWeight
import androidx.compose.ui.unit.dp
import androidx.compose.ui.unit.sp
import androidx.navigation.NavController
import java.io.File
import coil.compose.rememberAsyncImagePainter
import com.example.sdpapp.R

@Composable
fun PhotosScreen(navController: NavController) {
    Column(
        modifier = Modifier
            .fillMaxSize()
            .background(MaterialTheme.colorScheme.background)
            .padding(10.dp)
    ) {
        Text(
            text = "Photos",
            style = MaterialTheme.typography.bodyLarge,
            color = MaterialTheme.colorScheme.tertiary,
        )
        DisplayPhotos(navController)
    }
}


@Composable
fun DisplayPhotos(navController: NavController) {
    val context = LocalContext.current
    val itemNames = remember { getAllItems(context) }
    var images by remember { mutableStateOf(getAllImages(context, itemNames)) }
    var selectedImage by remember { mutableStateOf<File?>(null) }

    val groupedImages = itemNames.associateWith { category ->
        images.filter { it.first.equals(category) }.map { it.second }
    }.toMutableMap()

    itemNames.forEach { folder ->
        if (!groupedImages.containsKey(folder)) {
            groupedImages[folder] = emptyList()
        }
    }

    Box(modifier = Modifier.fillMaxSize()) {
        if (selectedImage == null) {
            LazyColumn(
                modifier = Modifier.fillMaxSize()
            ) {
                groupedImages.forEach { (category, files) ->
                    item {
                        Row(
                            modifier = Modifier
                                .fillMaxWidth()
                                .padding(bottom = 10.dp)
                                .height(60.dp),
                            horizontalArrangement = Arrangement.SpaceBetween,
                            verticalAlignment = Alignment.CenterVertically
                        ) {
                            Text(
                                text = category.replaceFirstChar { it.uppercase() },
                                color = MaterialTheme.colorScheme.surfaceBright,
                                fontSize = 27.sp,
                                fontWeight = FontWeight.Bold
                            )

                            var expanded by remember { mutableStateOf(false) }
                            var selectedOption by remember { mutableStateOf("Take Photo") }

                            val options = listOf("Take Photo", "Upload from Phone")

                            Box {
                                IconButton(
                                    onClick = { expanded = true }
                                ) {
                                    Icon(
                                        Icons.Filled.Add,
                                        contentDescription = "Add $category Photos",
                                        modifier = Modifier.size(35.dp),
                                        tint = MaterialTheme.colorScheme.surfaceBright
                                    )
                                }

                                DropdownMenu(
                                    expanded = expanded,
                                    onDismissRequest = { expanded = false }
                                ) {
                                    options.forEach { option ->
                                        DropdownMenuItem(
                                            text = {
                                                Text(
                                                    text = option,
                                                    fontSize = 18.sp
                                                )
                                            },
                                            onClick = {
                                                selectedOption = option
                                                expanded = false

                                                when (option) {
                                                    "Take Photo" -> {
                                                        navController.navigate("camera/$category")
                                                    }
                                                    "Upload from Phone" -> {
                                                        navController.navigate("upload/$category")
                                                    }
                                                }
                                            }
                                        )
                                    }
                                }
                            }
                        }

                    }

                    if (files.isNotEmpty()) {
                        item {
                            LazyRow(
                                modifier = Modifier.fillMaxWidth(),
                                contentPadding = PaddingValues(horizontal = 0.dp),
                                horizontalArrangement = Arrangement.spacedBy(0.dp)
                            ) {
                                items(files) { imageFile ->
                                    PhotoItem(imageFile) {
                                        selectedImage = imageFile
                                    }
                                }
                            }
                        }
                    } else {
                        item {
                            Text(
                                text = "No photos available",
                                color = Color.Gray,
                                fontSize = 16.sp,
                                modifier = Modifier.padding(16.dp)
                            )
                        }
                    }

                    item {
                        HorizontalDivider(
                            thickness = 1.dp,
                            color = MaterialTheme.colorScheme.surfaceBright,
                            modifier = Modifier.padding(top = 8.dp)
                        )
                    }
                }
            }
        }

        selectedImage?.let { image ->
            FullscreenImagePreview(
                image,
                onDelete = {
                    if (image.delete()) {
                        images = images.filterNot { it.second == image }
                    }
                    selectedImage = null
                },
                onClose = {
                    selectedImage = null
                }
            )
        }
    }
}


@Composable
fun PhotoItem(file: File, onClick: () -> Unit) {
    Column(
        modifier = Modifier.padding(0.dp),
        verticalArrangement = Arrangement.Center
    ) {
        Image(
            painter = rememberAsyncImagePainter(file),
            contentDescription = "Captured photo of ${file.parentFile?.name ?: "Unknown"}",
            contentScale = ContentScale.Crop,
            modifier = Modifier
                .size(220.dp)
                .padding(horizontal = 10.dp)
                .clip(RoundedCornerShape(0.dp))
                .clickable { onClick() }
        )
    }
}

@Composable
fun FullscreenImagePreview(imageFile: File, onDelete: () -> Unit, onClose: () -> Unit) {
    Box(
        modifier = Modifier.fillMaxSize()
    ) {
        Box(
            modifier = Modifier
                .fillMaxWidth()
                .background(MaterialTheme.colorScheme.background),
            contentAlignment = Alignment.TopCenter
        ) {
            Image(
                painter = rememberAsyncImagePainter(imageFile),
                contentDescription = "Full-screen preview",
                contentScale = ContentScale.Fit,
                modifier = Modifier.fillMaxSize().align(Alignment.TopCenter)
            )
        }

        Row(
            verticalAlignment = Alignment.CenterVertically,
            horizontalArrangement = Arrangement.Center,
            modifier = Modifier
                .fillMaxWidth()
                .padding(16.dp)
                .align(Alignment.BottomCenter)
        ) {
            Button(
                onClick = onDelete,
                modifier = Modifier
                    .padding(8.dp),
                colors = ButtonDefaults.buttonColors(
                    containerColor = MaterialTheme.colorScheme.error
                )
            ) {
                Text("Delete", color = Color.White)
            }

            Spacer(modifier = Modifier.width(16.dp))

            Button(
                onClick = onClose,
                modifier = Modifier
                    .padding(8.dp)
            ) {
                Text(
                    "Close",
                    color = MaterialTheme.colorScheme.surfaceBright
                )
            }
        }
    }
}


fun getAllImages(context: Context, itemNames: List<String>): List<Pair<String, File>> {
    val imageFiles = mutableListOf<Pair<String, File>>()
    val mainDir = context.filesDir

    itemNames.forEach { folderName ->
        val folder = File(mainDir, folderName)
        if (folder.exists() && folder.isDirectory) {
            val imagesInFolder = folder.listFiles()?.filter { it.extension in listOf("jpg", "png", "jpeg") }
            imagesInFolder?.forEach { file ->
                imageFiles.add(Pair(folderName, file))
            }
        }
    }

    return imageFiles
}

fun getAllItems(context: Context): List<String> {
    val mainDir = context.filesDir
    return mainDir.listFiles()?.filter { it.isDirectory }?.map { it.name } ?: emptyList()
}

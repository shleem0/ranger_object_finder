package com.example.sdpapp.ui

import android.content.Context
import androidx.compose.foundation.Image
import androidx.compose.foundation.background
import androidx.compose.foundation.layout.Arrangement
import androidx.compose.foundation.layout.Column
import androidx.compose.foundation.layout.PaddingValues
import androidx.compose.foundation.layout.fillMaxSize
import androidx.compose.foundation.layout.fillMaxWidth
import androidx.compose.foundation.layout.padding
import androidx.compose.foundation.layout.size
import androidx.compose.foundation.lazy.LazyColumn
import androidx.compose.foundation.lazy.LazyRow
import androidx.compose.foundation.lazy.grid.GridCells
import androidx.compose.foundation.lazy.grid.LazyVerticalGrid
import androidx.compose.foundation.lazy.grid.items
import androidx.compose.foundation.lazy.items
import androidx.compose.foundation.shape.RoundedCornerShape
import androidx.compose.material3.MaterialTheme
import androidx.compose.material3.Text
import androidx.compose.runtime.Composable
import androidx.compose.runtime.remember
import androidx.compose.ui.Modifier
import androidx.compose.ui.draw.clip
import androidx.compose.ui.layout.ContentScale
import androidx.compose.ui.platform.LocalContext
import androidx.compose.ui.res.painterResource
import androidx.compose.ui.res.stringResource
import androidx.compose.ui.text.font.FontWeight
import androidx.compose.ui.unit.dp
import androidx.compose.ui.unit.sp
import androidx.navigation.NavController
import com.example.sdpapp.FABWithNotification
import com.example.sdpapp.Item
import com.example.sdpapp.R
import java.io.File
import coil.compose.rememberAsyncImagePainter

@Composable
fun PhotosScreen (navController: NavController) {
    Column(
        modifier = Modifier
            .fillMaxSize()
            .background(MaterialTheme.colorScheme.background)
            .padding(top = 10.dp, start = 20.dp)
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
    val images = remember { getAllImages(context) }

    val groupedImages = images.groupBy { it.first }

    LazyColumn(
        modifier = Modifier.fillMaxSize()
    ) {
        groupedImages.forEach { (category, files) ->
            item {
                Text(
                    text = category.replaceFirstChar { it.uppercase() },
                    color = MaterialTheme.colorScheme.surfaceBright,
                    fontSize = 27.sp,
                    fontWeight = FontWeight.Bold,
                    modifier = Modifier.padding(top = 10.dp, start = 10.dp)
                )
            }

            item {
                LazyRow(
                    modifier = Modifier.fillMaxWidth(),
                    contentPadding = PaddingValues(horizontal = 0.dp),
                    horizontalArrangement = Arrangement.spacedBy(0.dp)
                ) {
                    items(files) { (_, imageFile) ->
                        PhotoItem(imageFile)
                    }
                }
            }
        }
    }
}

fun getAllImages(context: Context): List<Pair<String, File>> {
    val imageFiles = mutableListOf<Pair<String, File>>()
    val mainDir = context.filesDir

    mainDir.listFiles()?.forEach { subfolder ->
        if (subfolder.isDirectory) {
            subfolder.listFiles()?.forEach { file ->
                if (file.extension in listOf("jpg", "png", "jpeg")) {
                    imageFiles.add(Pair(subfolder.name, file))
                }
            }
        }
    }
    return imageFiles
}

@Composable
fun PhotoItem(file: File) {
    Column(
        modifier = Modifier.padding(0.dp),
        verticalArrangement = Arrangement.Center
    )
    {
        Image(
            painter = rememberAsyncImagePainter(file),
            contentDescription = "Captured photo",
            contentScale = ContentScale.Crop,
            modifier = Modifier
                .size(220.dp)
                .padding(horizontal = 10.dp)
                .clip(RoundedCornerShape(0.dp))
        )
    }
}
package com.example.sdpapp.ui.theme

import android.os.Build
import androidx.compose.foundation.isSystemInDarkTheme
import androidx.compose.material3.MaterialTheme
import androidx.compose.material3.darkColorScheme
import androidx.compose.material3.dynamicDarkColorScheme
import androidx.compose.material3.dynamicLightColorScheme
import androidx.compose.material3.lightColorScheme
import androidx.compose.ui.platform.LocalContext
import androidx.compose.runtime.Composable

private val DarkColourScheme = darkColorScheme(
    primary = AppColors.secondaryColor,
    secondary = AppColors.primaryColor,
    tertiary = AppColors.backgroundColor,
    background = AppColors.tertiaryColor,
    surfaceBright = AppColors.offWhite,
    onBackground = AppColors.tgtgColor
)

private val LightColourScheme = lightColorScheme(
    primary = AppColors.primaryColor,
    secondary = AppColors.secondaryColor,
    tertiary = AppColors.tertiaryColor,
    background = AppColors.backgroundColor,
    surfaceBright = AppColors.tgtgColor,
    onBackground = AppColors.offWhite
)

@Composable
fun SDPAppTheme(
    darkTheme: Boolean = isSystemInDarkTheme(),
    content: @Composable () -> Unit
) {
    val colorScheme = when {
        darkTheme -> DarkColourScheme
        else -> LightColourScheme
    }

    MaterialTheme(
        colorScheme = colorScheme,
        typography = Typography,
        content = content
    )
}
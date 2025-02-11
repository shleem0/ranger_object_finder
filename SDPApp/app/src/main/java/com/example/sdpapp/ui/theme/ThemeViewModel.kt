package com.example.sdpapp.ui.theme

import android.content.Context
import android.content.SharedPreferences
import androidx.lifecycle.ViewModel
import androidx.lifecycle.viewModelScope
import kotlinx.coroutines.flow.MutableStateFlow
import kotlinx.coroutines.flow.asStateFlow
import kotlinx.coroutines.launch

class ThemeViewModel(context: Context) : ViewModel() {
    private val preferences: SharedPreferences =
        context.getSharedPreferences("theme_prefs", Context.MODE_PRIVATE)

    private val _darkTheme = MutableStateFlow(preferences.getBoolean("dark_mode", false)) // Load from SharedPreferences
    val darkTheme = _darkTheme.asStateFlow()

    fun toggleTheme() {
        val newTheme = !_darkTheme.value
        _darkTheme.value = newTheme
        saveThemePreference(newTheme)
    }

    private fun saveThemePreference(isDarkMode: Boolean) {
        viewModelScope.launch {
            preferences.edit().putBoolean("dark_mode", isDarkMode).apply()
        }
    }
}
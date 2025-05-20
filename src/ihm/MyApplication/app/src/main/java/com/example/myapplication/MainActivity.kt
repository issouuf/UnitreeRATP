package com.example.myapplication


import android.os.Bundle
import android.webkit.WebSettings
import android.webkit.WebView
import android.webkit.WebViewClient
import androidx.appcompat.app.AppCompatActivity

class MainActivity : AppCompatActivity() {
    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        setContentView(R.layout.activity_main)

        val videoView = findViewById<WebView>(R.id.videoView)
        val webSettings: WebSettings = videoView.settings
        webSettings.javaScriptEnabled = true
        videoView.webViewClient = WebViewClient()

        // Remplace par l'URL de ton flux vidéo
        videoView.loadUrl("http://192.168.0.100:8080/video")
    }
}
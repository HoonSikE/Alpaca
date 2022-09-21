package com.example.taxi.di

import android.graphics.drawable.ColorDrawable
import android.os.Bundle
import androidx.appcompat.app.AppCompatActivity
import com.example.taxi.R
import com.example.taxi.databinding.ActivityMainBinding
import com.example.taxi.utils.view.toast
import dagger.hilt.android.AndroidEntryPoint

@AndroidEntryPoint
class MainActivity : AppCompatActivity() {
    lateinit var binding: ActivityMainBinding

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        binding = ActivityMainBinding.inflate(layoutInflater)
        setContentView(binding.root)
        supportActionBar?.let {
            it.setBackgroundDrawable(ColorDrawable(R.color.white))
            it.hide()
        }
    }

    override fun onBackPressed() {
        super.onBackPressed()
    }
}
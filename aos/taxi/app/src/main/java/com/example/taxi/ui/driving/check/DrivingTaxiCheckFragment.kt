package com.example.taxi.ui.driving.check

import android.Manifest
import android.app.Activity
import android.content.Intent
import android.content.pm.PackageManager
import android.graphics.*
import android.os.Build
import android.provider.MediaStore
import androidx.core.content.ContextCompat
import androidx.core.view.setPadding
import com.example.taxi.R
import com.example.taxi.base.BaseFragment
import com.example.taxi.databinding.FragmentDrivingTaxiCheckBinding
import com.example.taxi.utils.constant.show
import dagger.hilt.android.AndroidEntryPoint


@AndroidEntryPoint
class DrivingTaxiCheckFragment : BaseFragment<FragmentDrivingTaxiCheckBinding>(R.layout.fragment_driving_taxi_check) {

    private val REQUEST_IMAGE_CAPTURE = 1
    private var imageNum = 1

    override fun init() {
        checkPermission()
        setOnClickListeners()

    }

    private fun setOnClickListeners() {
        binding.imageItemMyPageAlbumImg.setOnClickListener {
            imageNum = 1
            onCamera()
        }
        binding.imageItemMyPageAlbumImg2.setOnClickListener {
            imageNum = 2
            onCamera()
        }
        binding.imageItemMyPageAlbumImg3.setOnClickListener {
            imageNum = 3
            onCamera()
        }
        binding.imageItemMyPageAlbumImg4.setOnClickListener {
            imageNum = 4
            onCamera()
        }
        binding.imageItemMyPageAlbumImg5.setOnClickListener {
            imageNum = 5
            onCamera()
        }
        binding.imageItemMyPageAlbumImg6.setOnClickListener {
            imageNum = 6
            onCamera()
        }
        binding.imageItemMyPageAlbumImg7.setOnClickListener {
            imageNum = 7
            onCamera()
        }
        binding.imageItemMyPageAlbumImg8.setOnClickListener {
            imageNum = 8
            onCamera()
        }
        binding.buttonDrivingTaxiCheck.setOnClickListener {
            //TODO : 이미지 업로드
            requireActivity().onBackPressed()
        }
    }

    private fun onCamera(){
        if(Build.VERSION.SDK_INT >= Build.VERSION_CODES.M)
            Intent(MediaStore.ACTION_IMAGE_CAPTURE).also { takePictureIntent ->
                takePictureIntent.resolveActivity(requireActivity().packageManager)?.also {
                    startActivityForResult(takePictureIntent, REQUEST_IMAGE_CAPTURE)
                }
            }
    }

    override fun onActivityResult(requestCode: Int, resultCode: Int, data: Intent?) {
        if (requestCode == REQUEST_IMAGE_CAPTURE && resultCode == Activity.RESULT_OK) {
            val imageBitmap = data!!.extras!!.get("data") as Bitmap
            when(imageNum){
                1 -> {
                    binding.imageItemMyPageAlbumImg.setImageBitmap(imageBitmap)
                    binding.imageItemMyPageAlbumImg2.show()
                    binding.imageItemMyPageAlbumImg.setPadding(0)
                }
                2 -> {
                    binding.imageItemMyPageAlbumImg2.setImageBitmap(imageBitmap)
                    binding.imageItemMyPageAlbumImg3.show()
                    binding.imageItemMyPageAlbumImg2.setPadding(0)
                }
                3 -> {
                    binding.imageItemMyPageAlbumImg3.setImageBitmap(imageBitmap)
                    binding.imageItemMyPageAlbumImg4.show()
                    binding.imageItemMyPageAlbumImg3.setPadding(0)
                }
                4 -> {
                    binding.imageItemMyPageAlbumImg4.setImageBitmap(imageBitmap)
                    binding.imageItemMyPageAlbumImg5.show()
                    binding.imageItemMyPageAlbumImg4.setPadding(0)
                }
                5 -> {
                    binding.imageItemMyPageAlbumImg5.setImageBitmap(imageBitmap)
                    binding.imageItemMyPageAlbumImg6.show()
                    binding.imageItemMyPageAlbumImg5.setPadding(0)
                }
                6 -> {
                    binding.imageItemMyPageAlbumImg6.setImageBitmap(imageBitmap)
                    binding.imageItemMyPageAlbumImg7.show()
                    binding.imageItemMyPageAlbumImg6.setPadding(0)
                }
                7 -> {
                    binding.imageItemMyPageAlbumImg7.setImageBitmap(imageBitmap)
                    binding.imageItemMyPageAlbumImg8.show()
                    binding.imageItemMyPageAlbumImg7.setPadding(0)
                }
                8 -> {
                    binding.imageItemMyPageAlbumImg8.setImageBitmap(imageBitmap)
                    binding.imageItemMyPageAlbumImg8.setPadding(0)
                }
            }
        }
    }

    private fun checkPermission() {
        var permission = mutableMapOf<String, String>()
        permission["camera"] = Manifest.permission.CAMERA

        var denied = permission.count { ContextCompat.checkSelfPermission(requireContext(), it.value)  == PackageManager.PERMISSION_DENIED }

        if(denied > 0 && Build.VERSION.SDK_INT >= Build.VERSION_CODES.M) {
            requestPermissions(permission.values.toTypedArray(), REQUEST_IMAGE_CAPTURE)
        }

    }

}
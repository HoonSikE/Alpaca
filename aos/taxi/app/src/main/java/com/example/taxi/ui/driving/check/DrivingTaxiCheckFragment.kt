package com.example.taxi.ui.driving.check

import android.Manifest
import android.app.Activity
import android.content.Context
import android.content.Intent
import android.content.pm.PackageManager
import android.graphics.*
import android.net.Uri
import android.os.Build
import android.provider.MediaStore
import android.util.Log
import androidx.core.app.ActivityCompat.startActivityForResult
import androidx.core.content.ContextCompat
import androidx.core.view.setPadding
import androidx.fragment.app.viewModels
import com.example.taxi.R
import com.example.taxi.base.BaseFragment
import com.example.taxi.data.dto.common.InsideCarList
import com.example.taxi.data.dto.common.PhotoList
import com.example.taxi.data.dto.user.destination.Destination
import com.example.taxi.databinding.FragmentDrivingTaxiCheckBinding
import com.example.taxi.di.ApplicationClass
import com.example.taxi.ui.driving.end.EndDrivingViewModel
import com.example.taxi.utils.constant.UiState
import com.example.taxi.utils.constant.hide
import com.example.taxi.utils.constant.show
import com.example.taxi.utils.view.toast
import dagger.hilt.android.AndroidEntryPoint
import retrofit2.http.Url
import java.io.ByteArrayOutputStream
import java.net.URI
import java.util.*


@AndroidEntryPoint
class DrivingTaxiCheckFragment : BaseFragment<FragmentDrivingTaxiCheckBinding>(R.layout.fragment_driving_taxi_check) {

    private val REQUEST_IMAGE_CAPTURE = 1
    private var imageNum = 1
    private var list: MutableList<String> = mutableListOf()
    private val endDrivingViewModel : EndDrivingViewModel by viewModels()
    lateinit var insideCarList: InsideCarList
    private var checkState = false

    override fun init() {
        initData()
        checkPermission()
        setOnClickListeners()
        observerData()
    }

    private fun initData() {
        if(arguments?.getBoolean("checkState")!=null){
            checkState = arguments?.getBoolean("checkState")!!
        }
    }

    private fun observerData() {
        endDrivingViewModel.insideCarList.observe(viewLifecycleOwner) { state ->
            when (state) {
                is UiState.Loading -> {
                    binding.progressBar.show()
                }
                is UiState.Failure -> {
                    binding.progressBar.hide()
                    state.error?.let {
                        toast(it)
                        Log.d("UiState.Failure", it)
                    }
                }
                is UiState.Success -> {
                    binding.progressBar.hide()
                    insideCarList = state.data
                    var check = false
                    for(i in insideCarList.photoList){
                        if(i.carNumber == ApplicationClass.prefs.carNumber){
                            if(checkState){
                                i.start = list
                            }else{
                                i.end = list
                            }
                            check = true
                        }
                    }
                    if(!check){
                        if(checkState){
                            insideCarList.photoList.add(PhotoList(ApplicationClass.prefs.carNumber.toString(), start = list))
                        }else{
                            insideCarList.photoList.add(PhotoList(ApplicationClass.prefs.carNumber.toString(), end = list))
                        }
                    }
                    endDrivingViewModel.addImageListUpLoad(imageNum, checkState, insideCarList.photoList)
                }
            }
        }
        endDrivingViewModel.photoList.observe(viewLifecycleOwner) {
            requireActivity().onBackPressed()
        }
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
            endDrivingViewModel.getInsideCarList()
        }
    }

    private fun onCamera(){
        if(Build.VERSION.SDK_INT >= Build.VERSION_CODES.LOLLIPOP)
            Intent(MediaStore.ACTION_IMAGE_CAPTURE).also { takePictureIntent ->
                takePictureIntent.resolveActivity(requireActivity().packageManager)?.also {
                    startActivityForResult(takePictureIntent, REQUEST_IMAGE_CAPTURE)
                }
            }
    }

    override fun onActivityResult(requestCode: Int, resultCode: Int, data: Intent?) {
        if (requestCode == REQUEST_IMAGE_CAPTURE && resultCode == Activity.RESULT_OK) {
            val imageBitmap = data!!.extras!!.get("data") as Bitmap
            val url : Uri? = getImageUri(requireContext(), imageBitmap)
            Log.d("photo url : ", url.toString())
            list.add(url.toString())
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

        if(denied > 0 && Build.VERSION.SDK_INT >= Build.VERSION_CODES.LOLLIPOP) {
            requestPermissions(permission.values.toTypedArray(), REQUEST_IMAGE_CAPTURE)
        }

    }

    fun getImageUri(inContext: Context, inImage: Bitmap?): Uri? {
        val bytes = ByteArrayOutputStream()
        if (inImage != null) {
            inImage.compress(Bitmap.CompressFormat.JPEG, 100, bytes)
        }
        val path = MediaStore.Images.Media.insertImage(inContext?.getContentResolver(), inImage, "Title" + " - " + Calendar.getInstance().getTime(), null)
        return Uri.parse(path)
    }


}
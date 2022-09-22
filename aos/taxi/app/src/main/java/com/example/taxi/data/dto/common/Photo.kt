package com.example.taxi.data.dto.common

import androidx.activity.result.ActivityResultLauncher
import com.canhub.cropper.CropImageContract
import com.canhub.cropper.CropImageContractOptions

data class Photo(
    var customCropImage: ActivityResultLauncher<CropImageContractOptions>? = null,
    var boolean: Boolean
)

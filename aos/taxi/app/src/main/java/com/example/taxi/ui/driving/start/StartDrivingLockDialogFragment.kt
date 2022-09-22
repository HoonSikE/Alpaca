package com.example.taxi.ui.driving.start

import android.annotation.SuppressLint
import android.graphics.Color
import android.graphics.drawable.ColorDrawable
import android.os.Bundle
import android.view.LayoutInflater
import android.view.View
import android.view.ViewGroup
import android.view.Window
import androidx.fragment.app.DialogFragment
import com.example.taxi.R
import com.example.taxi.databinding.DlgStartDrivingLockBinding
import com.ssafy.daero.utils.view.setTint

class StartDrivingLockDialogFragment(var lockState: Boolean) : DialogFragment() {

    private var _binding: DlgStartDrivingLockBinding? = null
    private val binding get() = _binding!!

    override fun onCreateView(
        inflater: LayoutInflater,
        container: ViewGroup?,
        savedInstanceState: Bundle?
    ): View? {
        _binding = DlgStartDrivingLockBinding.inflate(inflater, container, false)
        dialog?.window?.setBackgroundDrawable(ColorDrawable(Color.TRANSPARENT))
        dialog?.window?.requestFeature(Window.FEATURE_NO_TITLE)
        return binding.root
    }

    @SuppressLint("ResourceAsColor")
    override fun onViewCreated(view: View, savedInstanceState: Bundle?) {
        super.onViewCreated(view, savedInstanceState)
        if(lockState){
            binding.imageDlgStartDrivingLock.setColorFilter(resources.getColor(R.color.greenTextColor))
            binding.textDlgStartDrivingLockCheck.text = "문이 열렸습니다!"
            binding.textDlgStartDrivingLockCheck.setTextColor(resources.getColor(R.color.greenTextColor))
            binding.textDlgStartDrivingLockDescription.text = "차량에 탑승해주세요."
        }else{
            binding.imageDlgStartDrivingLock.setColorFilter(resources.getColor(R.color.redTextColor))
            binding.textDlgStartDrivingLockCheck.text = "문이 잠겼습니다!"
            binding.textDlgStartDrivingLockCheck.setTextColor(resources.getColor(R.color.redTextColor))
            binding.textDlgStartDrivingLockDescription.text = "탑승하시려면 문을 열어주세요."
        }
        setOnClickListeners()
    }

    private fun setOnClickListeners() {
        binding.buttonDlgStartDrivingLock.setOnClickListener { dismiss() }
    }

    override fun onDestroyView() {
        super.onDestroyView()
        _binding = null
    }

}
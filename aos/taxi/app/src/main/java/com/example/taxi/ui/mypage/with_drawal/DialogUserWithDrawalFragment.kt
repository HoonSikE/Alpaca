package com.example.taxi.ui.mypage.with_drawal

import android.os.Bundle
import android.view.LayoutInflater
import android.view.View
import android.view.ViewGroup
import androidx.fragment.app.DialogFragment
import androidx.fragment.app.viewModels
import com.example.taxi.databinding.DialogWithDrawalBinding
import com.example.taxi.ui.login.AuthViewModel
import com.example.taxi.utils.view.toast
import dagger.hilt.android.AndroidEntryPoint

@AndroidEntryPoint
class DialogUserWithDrawalFragment() : DialogFragment() {
    private var _binding: DialogWithDrawalBinding? = null
    private val binding get() = _binding!!

    private lateinit var listener : DialogWithDrawalOKClickedListener


    override fun onCreateView(inflater: LayoutInflater, container: ViewGroup?, savedInstanceState: Bundle?): View? {
        _binding = DialogWithDrawalBinding.inflate(inflater, container, false)
        val view = binding.root

        //ok 버튼 동작
        binding.buttonDialogUserWithDrawal.setOnClickListener {
            listener.onOKClicked(binding.buttonDialogUserWithDrawal.text.toString())
            dismiss()
        }
        return view
    }

    override fun onDestroyView() {
        super.onDestroyView()
        _binding = null
    }

    interface DialogWithDrawalOKClickedListener {
        fun onOKClicked(content : String)
    }

    fun setOnOKClickedListener(listener: (String) -> Unit) {
        this.listener = object: DialogWithDrawalOKClickedListener {
            override fun onOKClicked(content: String) {
                listener(content)
            }
        }
    }
}
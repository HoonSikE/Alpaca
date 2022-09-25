package com.example.taxi.ui.mypage.update_user

import android.app.Dialog
import android.content.Context
import android.os.Bundle
import android.view.LayoutInflater
import android.view.View
import android.view.ViewGroup
import android.view.Window
import androidx.appcompat.app.AppCompatActivity
import androidx.fragment.app.DialogFragment
import com.example.taxi.R
import com.example.taxi.databinding.DialogAddressBinding

class UpdateDialogAddressFragment(val title : String) : DialogFragment() {
    private var _binding: DialogAddressBinding? = null
    private val binding get() = _binding!!

    private lateinit var listener : DialogAddressOKClickedListener


    override fun onCreateView(inflater: LayoutInflater, container: ViewGroup?, savedInstanceState: Bundle?): View? {
        _binding = DialogAddressBinding.inflate(inflater, container, false)
        val view = binding.root

        if(title == "home"){
            binding.imageUpdateUserInfoHomeAddress.setImageResource(R.drawable.ic_home)
            binding.textDialogAddressTitle.text = "집 주소"
        }else if(title == "company"){
            binding.imageUpdateUserInfoHomeAddress.setImageResource(R.drawable.ic_company)
            binding.textDialogAddressTitle.text = "회사 주소"
        }

        //ok 버튼 동작
        binding.buttonDialogAddressUpdate.setOnClickListener {
            listener.onOKClicked(binding.edittextDialogAddressInput.text.toString())
            dismiss()
        }

        return view
    }

    override fun onDestroyView() {
        super.onDestroyView()
        _binding = null
    }

    interface DialogAddressOKClickedListener {
        fun onOKClicked(content : String)
    }

    fun setOnOKClickedListener(listener: (String) -> Unit) {
        this.listener = object: DialogAddressOKClickedListener {
            override fun onOKClicked(content: String) {
                listener(content)
            }
        }
    }
}
package com.example.taxi.ui.find.pw

import android.app.Dialog
import android.content.Context
import android.graphics.Color
import android.graphics.drawable.ColorDrawable
import android.os.Bundle
import android.text.Editable
import android.text.TextWatcher
import android.util.Log
import android.view.LayoutInflater
import android.view.View
import android.view.ViewGroup
import android.view.Window
import androidx.appcompat.app.AppCompatActivity
import androidx.fragment.app.DialogFragment
import androidx.recyclerview.widget.LinearLayoutManager
import androidx.recyclerview.widget.RecyclerView
import com.example.taxi.R
import com.example.taxi.data.api.KakaoAPI
import com.example.taxi.data.dto.user.destination.Destination
import com.example.taxi.data.dto.user.destination.DestinationSearch
import com.example.taxi.data.dto.user.destination.DestinationSearchDto
import com.example.taxi.databinding.DlgAddressBinding
import com.example.taxi.databinding.DlgEmailVerificationBinding
import com.example.taxi.ui.call_taxi.setting.DestinationSearchListAdapter
import com.example.taxi.utils.constant.KakaoApi
import retrofit2.Call
import retrofit2.Callback
import retrofit2.Response
import retrofit2.Retrofit
import retrofit2.converter.gson.GsonConverterFactory
import kotlin.math.acos
import kotlin.math.cos
import kotlin.math.roundToInt
import kotlin.math.sin

class FindPWDialogFragment(val title : String) : DialogFragment() {
    private var _binding: DlgEmailVerificationBinding? = null
    private val binding get() = _binding!!

    private lateinit var listener : FindPwDialogOKClickedListener

    override fun onCreateView(inflater: LayoutInflater, container: ViewGroup?, savedInstanceState: Bundle?): View? {
        _binding = DlgEmailVerificationBinding.inflate(inflater, container, false)
        dialog?.window?.setBackgroundDrawable(ColorDrawable(Color.TRANSPARENT))
        dialog?.window?.requestFeature(Window.FEATURE_NO_TITLE)

        val view = binding.root

        binding.textDlgEmailVerfication1.text = title

        //ok 버튼 동작
        binding.buttonDlgEmailVerfication.setOnClickListener {
            listener.onOKClicked(true)
            dismiss()
        }
        return view
    }

    override fun onDestroyView() {
        super.onDestroyView()
        _binding = null
    }

    interface FindPwDialogOKClickedListener {
        fun onOKClicked(content : Boolean)
    }

    fun setOnOKClickedListener(listener: (Boolean) -> Unit) {
        this.listener = object: FindPwDialogOKClickedListener {
            override fun onOKClicked(content: Boolean) {
                listener(content)
            }
        }
    }
}
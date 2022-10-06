package com.example.taxi.ui.mypage.update_user

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

class UpdateAddressDialogFragment(val title : String) : DialogFragment() {
    private var _binding: DlgAddressBinding? = null
    private val binding get() = _binding!!

    private lateinit var destinationSearchListAdapter: DestinationSearchListAdapter
    private lateinit var listener : AddressDialogOKClickedListener
    private lateinit var destination : Destination

    private val destinationSearchClickListener: (View, String, String, String, String) -> Unit = { _, place, address, x, y ->
        destination = Destination(address,y,place,x)
        binding.edittextDlgAddressInput.setText(destination.addressName)
    }

    override fun onCreateView(inflater: LayoutInflater, container: ViewGroup?, savedInstanceState: Bundle?): View? {
        _binding = DlgAddressBinding.inflate(inflater, container, false)
        dialog?.window?.setBackgroundDrawable(ColorDrawable(Color.TRANSPARENT))
        dialog?.window?.requestFeature(Window.FEATURE_NO_TITLE)

        val view = binding.root

        if(title == "home"){
            binding.imageUpdateUserInfoHomeAddress.setImageResource(R.drawable.ic_home)
            binding.textDlgAddressTitle.text = "집 주소"
        }else if(title == "company"){
            binding.imageUpdateUserInfoHomeAddress.setImageResource(R.drawable.ic_company)
            binding.textDlgAddressTitle.text = "회사 주소"
        }

        //ok 버튼 동작
        binding.buttonDlgAddressUpdate.setOnClickListener {
            listener.onOKClicked(binding.edittextDlgAddressInput.text.toString())
            dismiss()
        }

        binding.edittextDlgAddressInput.addTextChangedListener( object : TextWatcher {
            override fun beforeTextChanged(s: CharSequence?, start: Int, count: Int, after: Int) {}

            override fun onTextChanged(s: CharSequence?, start: Int, before: Int, count: Int) {
                if(binding.edittextDlgAddressInput.text.toString() == "") {
                    return
                }
                searchKeyword(binding.edittextDlgAddressInput.text.toString())
            }

            override fun afterTextChanged(s: Editable?) {}

        })

        return view
    }

    private fun initSearchAdapter(list : List<DestinationSearch>){
        destinationSearchListAdapter = DestinationSearchListAdapter().apply {
            onItemClickListener = destinationSearchClickListener
        }
        binding.recyclerDlgAddressUpdate.apply {
            adapter = destinationSearchListAdapter
            layoutManager = LinearLayoutManager(requireContext(), RecyclerView.VERTICAL, false)
        }
        destinationSearchListAdapter.updateList(list)
        binding.recyclerDlgAddressUpdate.visibility = View.VISIBLE
    }

    private fun searchKeyword(keyword: String) {
        val retrofit = Retrofit.Builder()   // Retrofit 구성
            .baseUrl(KakaoApi.BASE_URL)
            .addConverterFactory(GsonConverterFactory.create())
            .build()
        val api = retrofit.create(KakaoAPI::class.java)   // 통신 인터페이스를 객체로 생성
        val call = api.getSearchKeyword(KakaoApi.API_KEY, keyword)   // 검색 조건 입력

        // API 서버에 요청
        call.enqueue(object: Callback<DestinationSearchDto> {
            override fun onResponse(
                call: Call<DestinationSearchDto>,
                response: Response<DestinationSearchDto>
            ) {
                //통신 성공 (검색 결과는 response.body()에 담겨있음)
                Log.d("Test", "Raw: ${response.raw()}")
                Log.d("Test", "Body: ${response.body()}")
                if(response.body()!!.documents != null){
                    initSearchAdapter(response.body()!!.documents)
                }
            }

            override fun onFailure(call: Call<DestinationSearchDto>, t: Throwable) {
                // 통신 실패
                Log.w("MainActivity", "통신 실패: ${t.message}")
            }
        })
    }

    override fun onDestroyView() {
        super.onDestroyView()
        _binding = null
    }

    interface AddressDialogOKClickedListener {
        fun onOKClicked(content : String)
    }

    fun setOnOKClickedListener(listener: (String) -> Unit) {
        this.listener = object: AddressDialogOKClickedListener {
            override fun onOKClicked(content: String) {
                listener(content)
            }
        }
    }
}
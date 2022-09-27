package com.example.taxi.ui.driving.end

import android.util.Log
import androidx.fragment.app.viewModels
import androidx.navigation.fragment.findNavController
import com.bumptech.glide.Glide
import com.example.taxi.R
import com.example.taxi.base.BaseFragment
import com.example.taxi.data.dto.user.calltaxi.Taxi
import com.example.taxi.data.dto.user.calltaxi.TaxiList
import com.example.taxi.databinding.FragmentEndDrivingTaxiBinding
import com.example.taxi.di.ApplicationClass
import com.example.taxi.ui.call_taxi.CallTaxiViewModel
import com.example.taxi.utils.constant.UiState
import com.example.taxi.utils.constant.hide
import com.example.taxi.utils.view.toast
import dagger.hilt.android.AndroidEntryPoint

@AndroidEntryPoint
class EndDrivingTaxiFragment : BaseFragment<FragmentEndDrivingTaxiBinding>(R.layout.fragment_end_driving_taxi) {

    private val callTaxiViewModel : CallTaxiViewModel by viewModels()

    override fun init() {
        initData()
        setOnClickListeners()
        observerData()
    }

    private fun initData() {
        if(ApplicationClass.prefs.carImage != ""){
            Glide.with(this).load(ApplicationClass.prefs.carImage).into(binding.imageEndTaxiCar)
        }
        binding.textEndTaxi.text = ApplicationClass.prefs.carNumber
    }

    private fun setOnClickListeners(){

        binding.buttonEndTaxiPhoto.setOnClickListener {
            findNavController().navigate(R.id.action_endDrivingTaxiFragment_to_drivingTaxiCheckFragment)
        }
        binding.buttonEndTaxiStart.setOnClickListener {
            //사진 다 넣었는지 확인하기
            //TODO : RATINGBAR 값 업데이트
            //TODO : 후불일때 처리
            //TODO : 선불이거나, 후불 처리되면 다시 홈으로
            callTaxiViewModel.getTaxiList()
            findNavController().navigate(R.id.action_endDrivingTaxiFragment_to_userHomeFragment)
        }
    }

    private fun observerData() {
        callTaxiViewModel.taxiList.observe(viewLifecycleOwner) { state ->
            when (state) {
                is UiState.Loading -> {
                    //binding.progressBar.show()
                }
                is UiState.Failure -> {
                    //binding.progressBar.hide()
                    state.error?.let {
                        toast(it)
                        Log.d("UiState.Failure", it)
                    }
                }
                is UiState.Success -> {
                    sortTaxiList(state.data)
                }
            }
        }
        callTaxiViewModel.taxiListUpdate.observe(viewLifecycleOwner) { state ->
            when (state) {
                is UiState.Loading -> {
                    //binding.progressBar.show()
                }
                is UiState.Failure -> {
                    //binding.progressBar.hide()
                    state.error?.let {
                        toast(it)
                        Log.d("UiState.Failure", it)
                    }
                }
                is UiState.Success -> {
                    Log.d("UiState.Success", "taxiListUpdate clear")
                }
            }
        }
    }

    private fun sortTaxiList(taxiList: Taxi) {
        taxiList.isEachInOperation = false
        callTaxiViewModel.updateTaxiList(taxiList)
        //TODO : Provider isEachInOperation 업데이트

        //TODO : LastDestination, Destination(FrequentDestination) 업데이트
    }


}
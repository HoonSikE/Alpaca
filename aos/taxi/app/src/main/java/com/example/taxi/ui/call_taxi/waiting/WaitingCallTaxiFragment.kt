package com.example.taxi.ui.call_taxi.waiting

import android.util.Log
import androidx.core.os.bundleOf
import androidx.fragment.app.viewModels
import androidx.navigation.fragment.findNavController
import com.example.taxi.R
import com.example.taxi.base.BaseFragment
import com.example.taxi.data.dto.user.calltaxi.Taxi
import com.example.taxi.data.dto.user.calltaxi.TaxiList
import com.example.taxi.data.dto.user.destination.Destination
import com.example.taxi.databinding.FragmentWaitingCallTaxiBinding
import com.example.taxi.ui.call_taxi.CallTaxiViewModel
import com.example.taxi.utils.constant.UiState
import com.example.taxi.utils.constant.hide
import com.example.taxi.utils.view.toast
import dagger.hilt.android.AndroidEntryPoint

@AndroidEntryPoint
class WaitingCallTaxiFragment : BaseFragment<FragmentWaitingCallTaxiBinding>(R.layout.fragment_waiting_call_taxi) {

    private val callTaxiViewModel : CallTaxiViewModel by viewModels()
    private lateinit var startingPoint : Destination
    private lateinit var destination : Destination
    private lateinit var taxi : Taxi

    override fun init() {
        initData()
        observerData()
        setOnClickListeners()
    }

    private fun initData() {
        if(arguments?.getParcelable<Destination>("Destination")!=null && arguments?.getParcelable<Destination>("StartingPoint")!=null){
            destination = arguments?.getParcelable<Destination>("Destination") as Destination
            startingPoint = arguments?.getParcelable<Destination>("StartingPoint") as Destination
        }

        callTaxiViewModel.getTaxiList()
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
    }

    private fun setOnClickListeners(){
        binding.buttonWaitingCallTaxiCancel.setOnClickListener{
            requireActivity().onBackPressed()
        }
    }

    private fun sortTaxiList(taxiList: List<Taxi>) {
        //TODO : 가까운 순으로 정렬하기
        for(i in taxiList){
            Log.d("isEachDriving",i.isEachDriving.toString())
            Log.d("isEachInOperation",i.isEachInOperation.toString())
            if(i.isEachDriving && !i.isEachInOperation){
                taxi = i
                break
            }
        }
        binding.progressBarWaitingCallTaxiLoading.hide()
            Log.d("taxi", taxi.toString())
        //TODO : LastDestination, Destination(FrequentDestination) 업데이트
        findNavController().navigate(R.id.action_waitingCallTaxiFragment_to_assignedTaxiInformationFragment, bundleOf("Taxi" to taxi))
    }

}
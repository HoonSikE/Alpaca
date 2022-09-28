package com.example.taxi.ui.call_taxi.waiting

import android.util.Log
import androidx.core.os.bundleOf
import androidx.fragment.app.viewModels
import androidx.navigation.fragment.findNavController
import com.example.taxi.R
import com.example.taxi.base.BaseFragment
import com.example.taxi.data.dto.provider.ProviderCar
import com.example.taxi.data.dto.user.calltaxi.Taxi
import com.example.taxi.data.dto.user.calltaxi.TaxiList
import com.example.taxi.data.dto.user.destination.Destination
import com.example.taxi.data.dto.user.destination.FrequentDestination
import com.example.taxi.databinding.FragmentWaitingCallTaxiBinding
import com.example.taxi.di.ApplicationClass
import com.example.taxi.ui.call_taxi.CallTaxiViewModel
import com.example.taxi.ui.home.provider.ProviderViewModel
import com.example.taxi.ui.home.user.UserHomeViewModel
import com.example.taxi.utils.constant.UiState
import com.example.taxi.utils.constant.hide
import com.example.taxi.utils.view.toast
import dagger.hilt.android.AndroidEntryPoint

@AndroidEntryPoint
class WaitingCallTaxiFragment : BaseFragment<FragmentWaitingCallTaxiBinding>(R.layout.fragment_waiting_call_taxi) {

    private val callTaxiViewModel : CallTaxiViewModel by viewModels()
    private val providerViewModel : ProviderViewModel by viewModels()
    private val userHomeViewModel : UserHomeViewModel by viewModels()
    private lateinit var startingPoint : Destination
    private lateinit var destination : Destination
    private lateinit var taxi : Taxi
    private lateinit var providerCar : ProviderCar
    private var frequentDestination = mutableListOf<FrequentDestination>()
    private var destinations = mutableListOf<Destination>()

    override fun init() {
        initData()
        observerData()
        setOnClickListeners()
    }

    private fun initData() {
        callTaxiViewModel.getTaxiList()
        if(arguments?.getParcelable<Destination>("Destination")!=null && arguments?.getParcelable<Destination>("StartingPoint")!=null){
            destination = arguments?.getParcelable<Destination>("Destination") as Destination
            startingPoint = arguments?.getParcelable<Destination>("StartingPoint") as Destination
        }
        userHomeViewModel.getDestinations()
        userHomeViewModel.getLastDestinations()
    }

    private fun observerData() {
        userHomeViewModel.destinations.observe(viewLifecycleOwner) { state ->
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
                    //binding.progressBar.hide()
                    frequentDestination = state.data as MutableList<FrequentDestination>
                }
            }
        }
        userHomeViewModel.updateDestinations.observe(viewLifecycleOwner) { state ->
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
                    //binding.progressBar.hide()
                }
            }
        }
        userHomeViewModel.lastDestinations.observe(viewLifecycleOwner) { state ->
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
                    //binding.progressBar.hide()
                    destinations = state.data as MutableList<Destination>
                }
            }
        }
        userHomeViewModel.updateLastDestinations.observe(viewLifecycleOwner) { state ->
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
                    //binding.progressBar.hide()
                    findNavController().navigate(R.id.action_waitingCallTaxiFragment_to_assignedTaxiInformationFragment)
                }
            }
        }
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
        providerViewModel.provider.observe(viewLifecycleOwner) { state ->
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
                    providerCar = state.data.car!!
                    providerCar.isEachInOperation = true
                    providerViewModel.updateProvider(providerCar)
                }
            }
        }
    }

    private fun setOnClickListeners(){
        binding.buttonWaitingCallTaxiCancel.setOnClickListener{
            requireActivity().onBackPressed()
        }
    }

    private fun sortTaxiList(taxiList: Taxi) {
        taxi = taxiList
        binding.progressBarWaitingCallTaxiLoading.hide()
        Log.d("taxi", taxi.toString())
        ApplicationClass.prefs.carNumber = taxi.carNumber
        ApplicationClass.prefs.carImage = taxi.carImage
        ApplicationClass.prefs.rideComfortAverage = taxi.rideComfortAverage.toFloat()
        ApplicationClass.prefs.latitude = taxi.position.lati
        ApplicationClass.prefs.longitude = taxi.position.long
        ApplicationClass.prefs.cleanlinessAverage = taxi.cleanlinessAverage.toFloat()
        ApplicationClass.prefs.isEachDriving = taxi.isEachDriving
        ApplicationClass.prefs.isEachInOperation = !taxi.isEachInOperation
        ApplicationClass.prefs.startLatitude = startingPoint.latitude
        ApplicationClass.prefs.startLongitude = startingPoint.longitude
        ApplicationClass.prefs.startName = startingPoint.addressName
        ApplicationClass.prefs.destinationLatitude = destination.latitude
        ApplicationClass.prefs.destinationLongitude = destination.longitude
        ApplicationClass.prefs.destinationName = destination.addressName
        ApplicationClass.prefs.destinationAddress = destination.addressName
        ApplicationClass.prefs.providerId = taxi.userId
        ApplicationClass.prefs.carName = taxi.carName
        taxiList.isEachInOperation = true
        callTaxiViewModel.updateTaxiList(taxiList)
        providerViewModel.getProvider()
        var check = false
        for(i in frequentDestination){
            if(i.addressName == ApplicationClass.prefs.destinationName){
                i.count += 1
                check = true
                break
            }
        }
        if(!check){
            frequentDestination.add(FrequentDestination(ApplicationClass.prefs.destinationAddress.toString(), ApplicationClass.prefs.destinationLatitude.toString(),
                1,ApplicationClass.prefs.destinationName.toString(),ApplicationClass.prefs.destinationLongitude.toString()))
            userHomeViewModel.updateDestinations(frequentDestination)
        }
        check = false
        for(i in 0 until destinations.size){
            if(destinations[i].addressName == ApplicationClass.prefs.destinationName){
                val des = destinations[i]
                destinations.removeAt(i)
                destinations.add(des)
                check = true
                break
            }
        }
        if(!check){
            destinations.add(
                Destination(ApplicationClass.prefs.destinationAddress.toString(), ApplicationClass.prefs.destinationLatitude.toString()
                    ,ApplicationClass.prefs.destinationName.toString(),ApplicationClass.prefs.destinationLongitude.toString())
            )
            userHomeViewModel.updateLastDestinations(destinations)
        }
    }

}
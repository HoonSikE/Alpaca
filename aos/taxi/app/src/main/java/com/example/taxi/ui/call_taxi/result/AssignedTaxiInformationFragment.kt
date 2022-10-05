package com.example.taxi.ui.call_taxi.result

import android.util.Log
import androidx.core.os.bundleOf
import androidx.fragment.app.viewModels
import androidx.navigation.fragment.findNavController
import com.bumptech.glide.Glide
import com.example.taxi.R
import com.example.taxi.base.BaseFragment
import com.example.taxi.data.dto.provider.TaxiUser
import com.example.taxi.data.dto.provider.UserList
import com.example.taxi.data.dto.user.calltaxi.Taxi
import com.example.taxi.data.dto.user.destination.Destination
import com.example.taxi.data.dto.user.destination.FrequentDestination
import com.example.taxi.databinding.FragmentAssignedTaxiInformationBinding
import com.example.taxi.di.ApplicationClass
import com.example.taxi.ui.home.provider.ProviderViewModel
import com.example.taxi.ui.home.user.UserHomeViewModel
import com.example.taxi.utils.constant.UiState
import com.example.taxi.utils.constant.hide
import com.example.taxi.utils.constant.show
import com.example.taxi.utils.view.toast
import dagger.hilt.android.AndroidEntryPoint

@AndroidEntryPoint
class AssignedTaxiInformationFragment : BaseFragment<FragmentAssignedTaxiInformationBinding>(R.layout.fragment_assigned_taxi_information) {

    private val providerViewModel : ProviderViewModel by viewModels()
    private val userHomeViewModel : UserHomeViewModel by viewModels()
    private var frequentDestination = mutableListOf<FrequentDestination>()
    private var destination = mutableListOf<Destination>()


    override fun init() {
        initData()
        observerData()
        setOnClickListeners()
    }

    private fun initData() {
        binding.textAssignedTaxiInformationNumber.text = ApplicationClass.prefs.carNumber
        binding.ratingAssignedTaxiInformationRideComfort.rating = ApplicationClass.prefs.rideComfortAverage!!
        binding.ratingAssignedTaxiInformationCleanliness.rating = ApplicationClass.prefs.cleanlinessAverage!!
        if(ApplicationClass.prefs.carImage != ""){
            Glide.with(this).load(ApplicationClass.prefs.carImage).into(binding.imageAssignedTaxiInformationCar)
        }
        userHomeViewModel.getDestinations()
        userHomeViewModel.getLastDestinations()
    }

    private fun observerData() {
        userHomeViewModel.destinations.observe(viewLifecycleOwner) { state ->
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
                    frequentDestination = state.data as MutableList<FrequentDestination>
                }
            }
        }
        userHomeViewModel.updateDestinations.observe(viewLifecycleOwner) { state ->
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
                }
            }
        }
        userHomeViewModel.lastDestinations.observe(viewLifecycleOwner) { state ->
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
                    destination = state.data as MutableList<Destination>
                }
            }
        }
        userHomeViewModel.updateLastDestinations.observe(viewLifecycleOwner) { state ->
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
                }
            }
        }

    }

    private fun setOnClickListeners(){
        binding.buttonAssignedTaxiInformation.setOnClickListener{
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
            for(i in 0 until destination.size){
                if(destination[i].addressName == ApplicationClass.prefs.destinationName){
                    val des = destination[i]
                    destination.removeAt(i)
                    destination.add(des)
                    check = true
                    break
                }
            }
            if(!check){
                destination.add(
                    Destination(ApplicationClass.prefs.destinationAddress.toString(), ApplicationClass.prefs.destinationLatitude.toString()
                        ,ApplicationClass.prefs.destinationName.toString(),ApplicationClass.prefs.destinationLongitude.toString())
                )
                userHomeViewModel.updateLastDestinations(destination)
            }
            findNavController().navigate(R.id.action_assignedTaxiInformationFragment_to_locationTrackingTaxiFragment)
        }
    }

}
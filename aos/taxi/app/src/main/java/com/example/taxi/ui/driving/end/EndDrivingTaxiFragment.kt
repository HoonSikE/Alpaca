package com.example.taxi.ui.driving.end

import android.util.Log
import androidx.core.os.bundleOf
import androidx.fragment.app.viewModels
import androidx.navigation.fragment.findNavController
import com.bumptech.glide.Glide
import com.example.taxi.R
import com.example.taxi.base.BaseFragment
import com.example.taxi.data.dto.provider.ProviderCar
import com.example.taxi.data.dto.provider.TaxiUser
import com.example.taxi.data.dto.provider.UserList
import com.example.taxi.data.dto.user.boarded_taxi_list.BoardedTaxi
import com.example.taxi.data.dto.user.boarded_taxi_list.BoardedTaxiList
import com.example.taxi.data.dto.user.calltaxi.Taxi
import com.example.taxi.data.dto.user.calltaxi.TaxiList
import com.example.taxi.data.dto.user.destination.Destination
import com.example.taxi.data.dto.user.destination.FrequentDestination
import com.example.taxi.databinding.FragmentEndDrivingTaxiBinding
import com.example.taxi.di.ApplicationClass
import com.example.taxi.ui.call_taxi.CallTaxiViewModel
import com.example.taxi.ui.home.provider.ProviderViewModel
import com.example.taxi.ui.home.user.UserHomeViewModel
import com.example.taxi.utils.constant.UiState
import com.example.taxi.utils.constant.hide
import com.example.taxi.utils.constant.show
import com.example.taxi.utils.view.toast
import dagger.hilt.android.AndroidEntryPoint

@AndroidEntryPoint
class EndDrivingTaxiFragment : BaseFragment<FragmentEndDrivingTaxiBinding>(R.layout.fragment_end_driving_taxi) {

    private val callTaxiViewModel : CallTaxiViewModel by viewModels()
    private val providerViewModel : ProviderViewModel by viewModels()
    private val userHomeViewModel : UserHomeViewModel by viewModels()
    private val endDrivingViewModel : EndDrivingViewModel by viewModels()
    private var revenue: Int = 0
    private var fee = 0
    private lateinit var providerCar: ProviderCar
    private var frequentDestination = mutableListOf<FrequentDestination>()
    private var destinations = mutableListOf<Destination>()
    private var userList = mutableListOf<TaxiUser>()
    private var boardedTaxiList = mutableListOf<BoardedTaxi>()

    override fun init() {
        initData()
        setOnClickListeners()
        observerData()
    }

    private fun initData() {
        fee = ApplicationClass.prefs.fee!!
        if(ApplicationClass.prefs.carImage != ""){
            Glide.with(this).load(ApplicationClass.prefs.carImage).into(binding.imageEndTaxiCar)
        }
        binding.textEndTaxi.text = ApplicationClass.prefs.carNumber
        userHomeViewModel.getDestinations()
        userHomeViewModel.getLastDestinations()
        providerViewModel.getUserList()
        providerViewModel.getProvider()
        callTaxiViewModel.getTaxiList()
        endDrivingViewModel.getBoardedTaxiList()
    }

    private fun setOnClickListeners(){

        binding.buttonEndTaxiPhoto.setOnClickListener {
            findNavController().navigate(R.id.action_endDrivingTaxiFragment_to_drivingTaxiCheckFragment, bundleOf("checkState" to false))
        }
        binding.buttonEndTaxiStart.setOnClickListener {
            //사진 다 넣었는지 확인하기
            var resRide = 0.0
            var resClean = 0.0
            userList.add(TaxiUser(ApplicationClass.userId, binding.ratingEndTaxiRideComfort.rating.toDouble(),
                ApplicationClass.prefs.profileImage.toString(), binding.ratingEndTaxiCleanliness.rating.toDouble()))
            for(i in userList){
                resRide += i.rideComfort
                resClean += i.cleanliness
            }
            resRide /= userList.size
            resClean /= userList.size
            providerCar.cleanlinessAverage = resClean
            providerCar.rideComfortAverage = resRide
            providerViewModel.updateProvider(providerCar)
            var userLists = UserList(userList)
            providerViewModel.updateUserList(userLists)
            findNavController().navigate(R.id.action_endDrivingTaxiFragment_to_paymentFragment)
        }
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
                    destinations = state.data as MutableList<Destination>
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
        callTaxiViewModel.taxiList.observe(viewLifecycleOwner) { state ->
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
                    sortTaxiList(state.data)
                }
            }
        }
        callTaxiViewModel.taxiListUpdate.observe(viewLifecycleOwner) { state ->
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
                    Log.d("UiState.Success", "taxiListUpdate clear")
                }
            }
        }
        providerViewModel.provider.observe(viewLifecycleOwner) { state ->
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
                    Log.d("UiState.Success", "taxiListUpdate clear")
                    providerCar = state.data.car!!
                    revenue = state.data.revenue
                    revenue += ApplicationClass.prefs.fee!!
                    providerViewModel.updateRevenue(revenue)
                }
            }
        }
        providerViewModel.revenue.observe(viewLifecycleOwner) { state ->
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
                    Log.d("UiState.Success", "revenueUpdate clear")
                }
            }
        }
        providerViewModel.providerCar.observe(viewLifecycleOwner) { state ->
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
                    Log.d("UiState.Success", "revenueUpdate clear")
                }
            }
        }
        providerViewModel.userList.observe(viewLifecycleOwner) { state ->
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
                    userList = state.data.user as MutableList<TaxiUser>
                }
            }
        }
        providerViewModel.upadteUserList.observe(viewLifecycleOwner) { state ->
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
                    findNavController().navigate(R.id.action_endDrivingTaxiFragment_to_userHomeFragment)
                }
            }
        }
        endDrivingViewModel.boardedTaxiList.observe(viewLifecycleOwner) { state ->
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
                    boardedTaxiList = state.data.taxiList as MutableList<BoardedTaxi>
                }
            }
        }
        endDrivingViewModel.updateBoardedTaxiList.observe(viewLifecycleOwner) { state ->
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

    private fun sortTaxiList(taxiList: Taxi) {
        taxiList.isEachInOperation = false
        callTaxiViewModel.updateTaxiList(taxiList)
        boardedTaxiList.add(BoardedTaxi(fee, ApplicationClass.prefs.destinationName.toString(),
            ApplicationClass.prefs.startName.toString(), ApplicationClass.prefs.carImage.toString(),
            ApplicationClass.prefs.carName.toString(), ApplicationClass.prefs.carNumber.toString(),
            ApplicationClass.prefs.cleanlinessAverage!!.toDouble(), ApplicationClass.prefs.rideComfortAverage!!.toDouble(),
            ApplicationClass.prefs.distance!!.toDouble()
        ))
        endDrivingViewModel.updateBoardedTaxiList(boardedTaxiList)
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
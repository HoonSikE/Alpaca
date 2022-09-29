package com.example.taxi.ui.mypage.boarding_list.select_taxi_details.taxi_assessment

import android.util.Log
import androidx.core.os.bundleOf
import androidx.fragment.app.viewModels
import androidx.navigation.fragment.findNavController
import com.bumptech.glide.Glide
import com.example.taxi.R
import com.example.taxi.base.BaseFragment
import com.example.taxi.data.dto.user.boarded_taxi_list.BoardedTaxi
import com.example.taxi.data.dto.user.boarded_taxi_list.BoardedTaxiList
import com.example.taxi.databinding.FragmentTaxiAssessmentBinding
import com.example.taxi.ui.driving.end.EndDrivingViewModel
import com.example.taxi.utils.constant.UiState
import com.example.taxi.utils.view.toast
import com.gun0912.tedpermission.provider.TedPermissionProvider
import dagger.hilt.android.AndroidEntryPoint

@AndroidEntryPoint
class TaxiAssessmentFragment : BaseFragment<FragmentTaxiAssessmentBinding>(R.layout.fragment_taxi_assessment) {
    private val endDrivingViewModel : EndDrivingViewModel by viewModels()
    private lateinit var boardedTaxiList: BoardedTaxiList
    private lateinit var boardedTaxi: BoardedTaxi

    override fun init() {
        initData()
        setOnClickListeners()
        observerData()
    }

    private fun initData() {
        endDrivingViewModel.getBoardedTaxiList()

        if(arguments?.getParcelable<BoardedTaxi>("BoardedTaxi")!= null){
            boardedTaxi = arguments?.getParcelable<BoardedTaxi>("BoardedTaxi") as BoardedTaxi
            if(boardedTaxi.carImage != ""){
                Glide.with(TedPermissionProvider.context)
                    .load(boardedTaxi.carImage)
                    .into(binding.imageAssignedTaxiAssessmentCar)
            }else{
                Glide.with(TedPermissionProvider.context)
                    .load(R.drawable.img_car)
                    .into(binding.imageAssignedTaxiAssessmentCar)
            }
            binding.ratingAssignedTaxiAssessmentRideComfort.rating = boardedTaxi.rideComfortAverage.toFloat()
            binding.ratingAssignedTaxiAssessmentCleanliness.rating = boardedTaxi.cleanlinessAverage.toFloat()
        }
    }

    private fun setOnClickListeners() {
        binding.buttonAssignedTaxiAssessment.setOnClickListener{
            val index = arguments?.getInt("index") as Int

            boardedTaxi.rideComfortAverage = binding.ratingAssignedTaxiAssessmentRideComfort.rating.toDouble()
            boardedTaxiList.taxiList[index].rideComfortAverage = binding.ratingAssignedTaxiAssessmentRideComfort.rating.toDouble()
            boardedTaxi.cleanlinessAverage = binding.ratingAssignedTaxiAssessmentCleanliness.rating.toDouble()
            boardedTaxiList.taxiList[index].cleanlinessAverage = binding.ratingAssignedTaxiAssessmentCleanliness.rating.toDouble()


            endDrivingViewModel.updateBoardedTaxiList(boardedTaxiList.taxiList)

            findNavController().navigate(R.id.action_taxiAssessmentFragment_to_taxiDetailFragment, bundleOf("BoardedTaxi" to boardedTaxi, "index" to index))
        }
    }

    private fun observerData(){
        endDrivingViewModel.boardedTaxiList.observe(viewLifecycleOwner) { state ->
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
//                    boardedTaxiList = state.data.taxiList as MutableList<BoardedTaxi>
                    boardedTaxiList = state.data
                }
            }
        }
        endDrivingViewModel.updateBoardedTaxiList.observe(viewLifecycleOwner) { state ->
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
    }
}
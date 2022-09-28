package com.example.taxi.ui.mypage.boarding_list.select_taxi_details

import androidx.core.os.bundleOf
import androidx.navigation.fragment.findNavController
import com.bumptech.glide.Glide
import com.example.taxi.R
import com.example.taxi.base.BaseFragment
import com.example.taxi.data.dto.user.boarded_taxi_list.BoardedTaxi
import com.example.taxi.databinding.FragmentTaxiDetailBinding
import com.gun0912.tedpermission.provider.TedPermissionProvider
import dagger.hilt.android.AndroidEntryPoint

@AndroidEntryPoint
class TaxiDetailFragment : BaseFragment<FragmentTaxiDetailBinding>(R.layout.fragment_taxi_detail) {
    private lateinit var boardedTaxi: BoardedTaxi

    override fun init() {
        initData()
        setOnClickListeners()
//        observerData()
    }

    private fun initData() {
        if(arguments?.getParcelable<BoardedTaxi>("BoardedTaxi")!= null){
            boardedTaxi = arguments?.getParcelable<BoardedTaxi>("BoardedTaxi") as BoardedTaxi
            if(boardedTaxi.carImage != ""){
                Glide.with(TedPermissionProvider.context)
                    .load(boardedTaxi.carImage)
                    .into(binding.imageBoardingTaxiDetailCar)
            }else{
                Glide.with(TedPermissionProvider.context)
                    .load(R.drawable.img_car)
                    .into(binding.imageBoardingTaxiDetailCar)
            }
            binding.ratingBoardingTaxiDetailRideComfort.rating = boardedTaxi.rideComfortAverage.toFloat()
            binding.ratingBoardingTaxiDetailCleanliness.rating = boardedTaxi.cleanlinessAverage.toFloat()
            binding.textBoardingTaxiStart.text = boardedTaxi.StartingPoint
            binding.textBoardingTaxiDestination.text = boardedTaxi.Destination
            binding.textBoardingTaxiTimeRequired.text = boardedTaxi.timeRequired
            binding.textBoardingTaxiDistance.text = boardedTaxi.distance.toString() + "km"
            binding.textBoardingTaxiCost.text = boardedTaxi.Cost.toString() + "원"
        }
    }

    private fun setOnClickListeners() {
        binding.layoutBoardingTaxiDetailAssessment.setOnClickListener{
            val idx = arguments?.getInt("index") as Int
            findNavController().navigate(R.id.action_taxiDetailFragment_to_taxiAssessmentFragment, bundleOf("BoardedTaxi" to boardedTaxi, "index" to idx))
        }
        binding.buttonBoardingTaxiPersonChat.setOnClickListener{
            findNavController().navigate(R.id.action_taxiDetailFragment_to_personalChatFragment)
        }
    }
}
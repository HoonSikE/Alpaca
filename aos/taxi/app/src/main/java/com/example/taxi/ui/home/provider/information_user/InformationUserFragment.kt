package com.example.taxi.ui.home.provider.information_user

import androidx.navigation.fragment.findNavController
import com.example.taxi.R
import com.example.taxi.base.BaseFragment
import com.example.taxi.data.dto.provider.TaxiUser
import com.example.taxi.data.dto.user.destination.Destination
import com.example.taxi.databinding.FragmentInformationUserBinding
import dagger.hilt.android.AndroidEntryPoint

@AndroidEntryPoint
class InformationUserFragment : BaseFragment<FragmentInformationUserBinding>(R.layout.fragment_information_user) {

    private lateinit var taxiUser: TaxiUser
    //TODO : 어댑터 만들기

    override fun init() {
        initData()
        observerData()
        setOnClickListeners()
    }
    private fun initData(){
        if(arguments?.getParcelable<Destination>("TaxiUser")!=null){
            taxiUser = arguments?.getParcelable<TaxiUser>("TaxiUser") as TaxiUser
            binding.ratingInformationUserRideComfort.rating = taxiUser.rideComfort.toFloat()
            binding.ratingInformationUserCleanliness.rating = taxiUser.cleanliness.toFloat()
        }
    }

    private fun observerData() {
        //TODO : 사진받아오기
    }
    private fun setOnClickListeners() {
        binding.buttonInformationUserStart.setOnClickListener {
            findNavController().navigate(R.id.action_informationUserFragment_to_personalChatFragment)
        }
    }
}
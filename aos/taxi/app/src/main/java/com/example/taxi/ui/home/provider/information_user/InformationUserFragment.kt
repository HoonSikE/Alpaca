package com.example.taxi.ui.home.provider.information_user

import android.util.Log
import androidx.fragment.app.viewModels
import androidx.navigation.fragment.findNavController
import androidx.recyclerview.widget.LinearLayoutManager
import androidx.recyclerview.widget.RecyclerView
import com.bumptech.glide.Glide
import com.example.taxi.R
import com.example.taxi.base.BaseFragment
import com.example.taxi.data.dto.common.InsideCarList
import com.example.taxi.data.dto.common.PhotoList
import com.example.taxi.data.dto.provider.TaxiUser
import com.example.taxi.data.dto.user.destination.Destination
import com.example.taxi.databinding.FragmentInformationUserBinding
import com.example.taxi.di.ApplicationClass
import com.example.taxi.ui.driving.end.EndDrivingViewModel
import com.example.taxi.utils.constant.UiState
import com.example.taxi.utils.constant.hide
import com.example.taxi.utils.constant.show
import com.example.taxi.utils.view.toast
import dagger.hilt.android.AndroidEntryPoint

@AndroidEntryPoint
class InformationUserFragment : BaseFragment<FragmentInformationUserBinding>(R.layout.fragment_information_user) {

    private lateinit var taxiUser: TaxiUser
    private lateinit var informationUserAdapter : InformationUserAdapter
    private lateinit var informationUserEndAdapter : InformationUserAdapter
    private val endDrivingViewModel : EndDrivingViewModel by viewModels()
    private lateinit var inside : InsideCarList

    override fun init() {
        initData()
        observerData()
        setOnClickListeners()
    }
    private fun initData(){
        if(arguments?.getParcelable<TaxiUser>("TaxiUser")!=null){
            taxiUser = arguments?.getParcelable<TaxiUser>("TaxiUser") as TaxiUser
            if(taxiUser.userImage!=""){
                Glide.with(requireContext())
                    .load(taxiUser.userImage)
                    .into(binding.imageInformationInformationUser)
            }
            binding.ratingInformationUserRideComfort.rating = taxiUser.rideComfort.toFloat()
            binding.ratingInformationUserCleanliness.rating = taxiUser.cleanliness.toFloat()
            endDrivingViewModel.getInsideCarList()
        }
    }

    private fun observerData() {
        endDrivingViewModel.insideCarList.observe(viewLifecycleOwner) { state ->
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
                    inside = state.data
                    initAdapter()
                }
            }
        }
    }

    private fun initAdapter() {
        for(i in inside.photoList){
            if(i.carNumber == ApplicationClass.prefs.carNumber){
                informationUserAdapter = InformationUserAdapter().apply {
                    photoList = i.start
                }
                informationUserEndAdapter = InformationUserAdapter().apply {
                    photoList = i.end
                }
                break
            }
        }
        binding.recyclerviewInformationStart.apply {
            adapter = informationUserAdapter
            layoutManager = LinearLayoutManager(requireContext(), RecyclerView.HORIZONTAL, false)
        }
        binding.recyclerviewInformationEnd.apply {
            adapter = informationUserEndAdapter
            layoutManager = LinearLayoutManager(requireContext(), RecyclerView.HORIZONTAL, false)
        }
    }

    private fun setOnClickListeners() {
        binding.buttonInformationUserStart.setOnClickListener {
            findNavController().navigate(R.id.action_informationUserFragment_to_personalChatFragment)
        }
        binding.imageInformationUserBack.setOnClickListener {
            requireActivity().onBackPressed()
        }
    }
}
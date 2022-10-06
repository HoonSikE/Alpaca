package com.example.taxi.ui.home.provider

import android.util.Log
import android.view.View
import android.widget.CompoundButton
import androidx.core.os.bundleOf
import androidx.fragment.app.viewModels
import androidx.navigation.fragment.findNavController
import androidx.recyclerview.widget.LinearLayoutManager
import androidx.recyclerview.widget.RecyclerView
import com.bumptech.glide.Glide
import com.example.taxi.R
import com.example.taxi.base.BaseFragment
import com.example.taxi.data.dto.provider.Provider
import com.example.taxi.data.dto.provider.TaxiUser
import com.example.taxi.data.dto.provider.UserList
import com.example.taxi.databinding.FragmentProviderHomeBinding
import com.example.taxi.di.ApplicationClass
import com.example.taxi.utils.constant.UiState
import com.example.taxi.utils.constant.hide
import com.example.taxi.utils.constant.show
import com.example.taxi.utils.view.toast
import dagger.hilt.android.AndroidEntryPoint
import java.text.NumberFormat


@AndroidEntryPoint
class ProviderHomeFragment : BaseFragment<FragmentProviderHomeBinding>(R.layout.fragment_provider_home) {

    private val providerViewModel : ProviderViewModel by viewModels()
    private lateinit var provider : Provider
    private lateinit var providerAdapter: ProviderAdapter
    private lateinit var userList: UserList

    private val onClickListener: (View, Int) -> Unit = { _, idx ->
        findNavController().navigate(R.id.action_providerHomeFragment_to_informationUserFragment, bundleOf("TaxiUser" to userList.user[idx]))
    }


    override fun init() {
        initData()
        observerData()
        setOnClickListeners()
    }

    private fun initData(){
        providerViewModel.getProvider()
        providerViewModel.getUserList()
    }

    private fun observerData() {
        providerViewModel.userList.observe(viewLifecycleOwner) { state ->
            when (state) {
                is UiState.Loading -> {
//                    binding.progressBar.show()
                    binding.textProviderHomeNoContentUserList.show()
                    binding.textProviderHomeFailedUserList.hide()
                }
                is UiState.Failure -> {
//                    binding.progressBar.hide()
                    binding.textProviderHomeNoContentUserList.hide()
                    binding.textProviderHomeFailedUserList.show()
                    state.error?.let {
                        toast(it)
                        Log.d("UiState.Failure", it)
                    }
                }
                is UiState.Success -> {
//                    binding.progressBar.hide()
                    binding.textProviderHomeNoContentUserList.hide()
                    binding.textProviderHomeFailedUserList.hide()
                        userList = state.data
                        initAdapter(state.data)

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
                    provider = state.data
                    initView()
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
                    Log.d("UiState.Success", "providerCar clear")
                }
            }
        }
    }

    private fun setOnClickListeners() {
        binding.textProviderHomeDeadLine.setOnClickListener {
            showDialog()
            providerViewModel.getProvider()
        }
        binding.imageUserHomeBack.setOnClickListener {
            requireActivity().onBackPressed()
        }
        binding.switchProviderHomeIsEachDriving.setOnCheckedChangeListener(
            CompoundButton.OnCheckedChangeListener { buttonView, isChecked ->
            if (isChecked) {
                provider.car!!.isEachDriving = true
                providerViewModel.updateProvider(provider.car!!)
            } else {
                provider.car!!.isEachDriving = false
                providerViewModel.updateProvider(provider.car!!)
            }
        })
    }

    private fun initView(){
        val numberFormat: NumberFormat = NumberFormat.getInstance()
            val str = numberFormat.format(provider.revenue)
        binding.textProviderHomeName.text = ApplicationClass.prefs.name+"님, 안녕하세요!"
        if(ApplicationClass.prefs.profileImage != ""){
            Glide.with(requireContext())
                .load(ApplicationClass.prefs.profileImage)
                .into(binding.imageProviderHomeProfile)
        }
        binding.textProviderHomeCount.text = str.toString() + "원"
        Glide.with(requireContext())
            .load(provider.car!!.carImage)
            .into(binding.imageProviderHomeCar)
        binding.textProviderHomeNumber.text = provider.car!!.carNumber
        binding.ratingProviderHomeRideComfort.rating = provider.car!!.rideComfortAverage.toFloat()
        binding.ratingProviderHomeCleanliness.rating = provider.car!!.cleanlinessAverage.toFloat()
        binding.switchProviderHomeIsEachDriving.isChecked = provider.car!!.isEachDriving
        binding.textProviderHomeDeadLine.text = provider.car!!.deadLine
    }

    private fun initAdapter(userList: UserList) {
        providerAdapter = ProviderAdapter().apply {
            onTaxiUserClickListener = onClickListener
            context = requireContext()
            if(userList.user.isNotEmpty()){
                taxiUserList = userList.user as MutableList<TaxiUser>
            }
        }
        binding.recyclerProviderHomeUserList.apply {
            adapter = providerAdapter
            layoutManager = LinearLayoutManager(requireContext(), RecyclerView.HORIZONTAL, false)
        }
    }

    private fun showDialog() {
            ProviderDialogFragment(providerListener).show(childFragmentManager, "ProviderDialogFragment")
    }

    private val providerListener: () -> Unit = {
        providerViewModel.getProvider()
    }

}